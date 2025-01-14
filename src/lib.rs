//! # Giant Magellan Telescope Linear Optical Model
//!
//! Optical linear transformations applied to the rigid body motions of the primary and secondary segmented mirrors of the GMT
//!
//! The optical sensitivities can be downloaded from [here](https://s3.us-west-2.amazonaws.com/gmto.modeling/optical_sensitivities.rs.bin),
//! or they can be recomputed with the [makesens](../makesens/index.html) binary compiled with the `crseo` features
//! and run on a computer with a NVIDIA GPU.
//!
//! # Example
//! ```
//! use std::iter::once;
//! use skyangle::Conversion;
//! use gmt_lom::LOM;
//!
//! let m1_rbm = vec![vec![0f64; 6]; 7];
//! let mut m2_rbm = vec![vec![0f64; 6]; 7];
//! m2_rbm
//!     .iter_mut()
//!     .step_by(2)
//!     .for_each(|str| {
//!        str[3] = 1f64.from_arcsec();
//!        str[4] = 1f64.from_arcsec();
//!  });
//! m2_rbm[6][3] = 1f64.from_arcsec();
//! m2_rbm[6][4] = 1f64.from_arcsec();
//! let lom = LOM::builder()
//!     .into_iter_rigid_body_motions(once((m1_rbm, m2_rbm)))
//!     .build()
//!     .unwrap();
//! let tt = lom.tiptilt();
//! println!(" Tiptilt: {:.0?}mas", tt);
//! let stt = lom.segment_tiptilt();
//! println!("Segment tiptilt:");
//! println!(" - x: {:.0?}mas", &stt[..7]);
//! println!(" - y: {:.0?}mas", &stt[7..]);
//! let sp = lom.segment_piston();
//! println!("Segment piston:");
//! sp.chunks(7).enumerate().for_each(|(k, sp)| println!(" - S{}: {:.0?}nm", k + 1, sp) );
//! ```

use bincode;
use serde::Serialize;
use serde_pickle as pickle;

use std::{
    env,
    fs::File,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    path::{Path, PathBuf},
    slice::Chunks,
};

pub mod lom;
pub use lom::{LOMBuilder, LOM};
mod optical_sensitivities;
pub use optical_sensitivities::{from_opticals, OpticalSensitivities, OpticalSensitivity};
mod rigid_body_motions;
pub use rigid_body_motions::RigidBodyMotions;
#[cfg(feature = "apache")]
mod table;
#[cfg(feature = "apache")]
pub use table::Table;
// pub mod actors_interface;

#[derive(thiserror::Error, Debug)]
pub enum LinearOpticalModelError {
    #[error("sensitivities file not found (optical_sensitivities.rs.bin)")]
    SensitivityFile(#[from] std::io::Error),
    #[error("sensitivities cannot be loaded from optical_sensitivities.rs.bin")]
    SensitivityData(#[from] bincode::Error),
    #[error("segment tip-tilt sensitivity is missing")]
    SegmentTipTilt,
    #[error("rigid body motions are missing")]
    MissingRigidBodyMotions,
    #[error("failed to write optical metric to pickle file ")]
    MetricsPickleFile(#[from] pickle::Error),
    #[cfg(feature = "apache")]
    #[error("parquet read failed")]
    Parquet(#[from] parquet::errors::ParquetError),
    #[cfg(feature = "apache")]
    #[error("arrow record get failed")]
    Arrow(#[from] arrow::error::ArrowError),
    #[error("missing table {0} column ")]
    Table(String),
    #[error("PMT read failed")]
    Read(#[from] csv::Error),
}
type Result<T> = std::result::Result<T, LinearOpticalModelError>;

#[derive(Debug, Clone)]
pub enum Formatting {
    AdHoc,
    Latex,
}

/// Sensitivities serialization into a [bincode] file
pub trait Bin {
    fn dump<P: AsRef<Path>>(self, path: P) -> Result<Self>
    where
        Self: Sized;
    fn load<P: AsRef<Path>>(path: P) -> Result<Self>
    where
        Self: Sized;
}
impl<const N: usize> Bin for OpticalSensitivities<N> {
    /// Saves sensitivities to `path`
    fn dump<P: AsRef<Path>>(self, path: P) -> Result<Self> {
        bincode::serialize_into(File::create(path)?, &self)?;
        Ok(self)
    }
    /// Load sensitivities from `path`
    fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        Ok(bincode::deserialize_from(File::open(path)?)?)
    }
}

/// Data loader
pub struct Loader<T> {
    path: PathBuf,
    filename: String,
    phantom: PhantomData<T>,
}
/// [Loader] loading interface
pub trait LoaderTrait<T> {
    fn load(self) -> Result<T>;
}
impl<T> Loader<T> {
    /// Set the loading path
    pub fn path<P: AsRef<Path>>(self, path: P) -> Self {
        Self {
            path: Path::new(path.as_ref()).to_path_buf(),
            ..self
        }
    }
    /// Set the loaded file name
    pub fn filename(self, filename: &str) -> Self {
        Self {
            filename: String::from(filename),
            ..self
        }
    }
}
impl<const N: usize> Default for Loader<OpticalSensitivities<N>> {
    /// Default [Loader] for [Vec] of [OpticalSensitivity],
    /// expecting the file `optical_sensitivities.rs.bin` in the current folder
    fn default() -> Self {
        let path = env::var("LOM").unwrap_or_else(|_| ".".to_string());
        Self {
            path: Path::new(&path).to_path_buf(),
            filename: String::from("optical_sensitivities.rs.bin"),
            phantom: PhantomData,
        }
    }
}
impl<const N: usize> LoaderTrait<OpticalSensitivities<N>> for Loader<OpticalSensitivities<N>> {
    /// Loads precomputed optical sensitivities
    fn load(self) -> Result<OpticalSensitivities<N>> {
        println!("Loading optical sensitivities ...");
        <OpticalSensitivities<N> as Bin>::load(self.path.join(self.filename))
    }
}
#[cfg(feature = "apache")]
impl Default for Loader<RigidBodyMotions> {
    /// Default [Loader] for [RigidBodyMotions] expecting the file `data.parquet` in the current folder
    fn default() -> Self {
        Self {
            path: Path::new(".").to_path_buf(),
            filename: String::from("data.parquet"),
            phantom: PhantomData,
        }
    }
}
#[cfg(feature = "apache")]
impl LoaderTrait<RigidBodyMotions> for Loader<RigidBodyMotions> {
    /// Loads M1 and M2 rigid body motions
    fn load(self) -> Result<RigidBodyMotions> {
        println!("Loading rigid body motions ...");
        RigidBodyMotions::from_parquet(self.path.join(self.filename), None, None)
    }
}

/// Type holding the tip-tilt values
#[derive(Serialize, Debug, Clone)]
pub struct TipTilt(Vec<f64>);
/// Type holding the segment tip-tilt values
#[derive(Serialize, Debug, Clone)]
pub struct SegmentTipTilt(Vec<f64>);
/// Type holding the segment piston values
#[derive(Serialize, Debug, Clone)]
pub struct SegmentPiston(Vec<f64>);
// Dereferencing
impl Deref for TipTilt {
    type Target = Vec<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for TipTilt {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl Deref for SegmentTipTilt {
    type Target = Vec<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for SegmentTipTilt {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl Deref for SegmentPiston {
    type Target = Vec<f64>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for SegmentPiston {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl From<TipTilt> for Vec<f64> {
    fn from(value: TipTilt) -> Self {
        value.0
    }
}
impl From<SegmentPiston> for Vec<f64> {
    fn from(value: SegmentPiston) -> Self {
        value.0
    }
}
impl From<SegmentTipTilt> for Vec<f64> {
    fn from(value: SegmentTipTilt) -> Self {
        value.0
    }
}

pub trait ToPkl {
    /// Writes optical metrics to a [pickle] file
    fn to_pkl<P: AsRef<Path>>(&self, path: P) -> Result<()>
    where
        Self: Serialize + Sized,
    {
        let mut file = File::create(&path)?;
        pickle::ser::to_writer(&mut file, self, pickle::ser::SerOptions::new())?;
        Ok(())
    }
}
impl ToPkl for TipTilt {}
impl ToPkl for SegmentTipTilt {}
impl ToPkl for SegmentPiston {}

/// Trait for the [LOM] optical metrics
///
/// A simple trait looking at the number of items in the [TipTilt], [SegmentTipTilt] and [SegmentPiston] metrics
pub trait OpticalMetrics {
    fn n_item(&self) -> usize;
    /// Returns a [chunks] iterator with chunks the size of [n_item]
    fn items(&self) -> Chunks<'_, f64>
    where
        Self: Deref<Target = Vec<f64>>,
    {
        self.deref().chunks(self.n_item())
    }
    /// Returns the metrics assigning each component in a contiguous time vector
    fn time_wise(&self, n_sample: Option<usize>) -> Vec<f64>;
}
impl OpticalMetrics for TipTilt {
    /// [TipTilt] `2` x and y items
    fn n_item(&self) -> usize {
        2
    }
    fn time_wise(&self, n_sample: Option<usize>) -> Vec<f64> {
        let n_item = self.n_item();
        let n_total = self.len() / n_item;
        assert!(n_total >= n_sample.unwrap_or(n_total), "not enough samples");
        (0..n_item)
            .flat_map(|i| {
                self.iter()
                    .skip(i)
                    .step_by(n_item)
                    .skip(n_total - n_sample.unwrap_or(n_total))
            })
            .cloned()
            .collect()
    }
}
impl OpticalMetrics for SegmentTipTilt {
    /// [SegmentTipTilt] `7x2` x and y items
    fn n_item(&self) -> usize {
        14
    }
    fn time_wise(&self, n_sample: Option<usize>) -> Vec<f64> {
        let n_item = self.n_item();
        let n_total = self.len() / n_item;
        assert!(n_total >= n_sample.unwrap_or(n_total), "not enough samples");
        (0..n_item)
            .flat_map(|i| {
                self.iter()
                    .skip(i)
                    .step_by(n_item)
                    .skip(n_total - n_sample.unwrap_or(n_total))
            })
            .cloned()
            .collect()
    }
}
impl OpticalMetrics for SegmentPiston {
    /// [SegmentPiston] `7` items
    fn n_item(&self) -> usize {
        7
    }
    fn time_wise(&self, n_sample: Option<usize>) -> Vec<f64> {
        let n_item = self.n_item();
        let n_total = self.len() / n_item;
        assert!(n_total >= n_sample.unwrap_or(n_total), "not enough samples");
        (0..n_item)
            .flat_map(|i| {
                self.iter()
                    .skip(i)
                    .step_by(n_item)
                    .skip(n_total - n_sample.unwrap_or(n_total))
            })
            .cloned()
            .collect()
    }
}

/// Statistics on [OpticalMetrics]
pub trait Stats {
    /// Returns the mean values
    ///
    /// Optionally, the statistical moment is evaluated on the last `n_sample`
    fn mean(&self, n_sample: Option<usize>) -> Vec<f64>
    where
        Self: Deref<Target = Vec<f64>> + OpticalMetrics,
    {
        let n_item = self.n_item();
        let n_total = self.len() / n_item;
        assert!(n_total >= n_sample.unwrap_or(n_total), "not enough samples");
        (0..n_item)
            .map(|i| {
                self.iter()
                    .skip(i)
                    .step_by(n_item)
                    .skip(n_total - n_sample.unwrap_or(n_total))
                    .sum::<f64>()
                    / n_sample.unwrap_or(n_total) as f64
            })
            .collect()
    }
    /// Returns the mean variance values
    ///
    /// Optionally, the statistical moment is evaluated on the last `n_sample`
    fn var(&self, n_sample: Option<usize>) -> Vec<f64>
    where
        Self: Deref<Target = Vec<f64>> + OpticalMetrics,
    {
        let n_item = self.n_item();
        let n_total = self.len() / n_item;
        assert!(n_total >= n_sample.unwrap_or(n_total), "not enough samples");
        (0..n_item)
            .zip(self.mean(n_sample))
            .map(|(i, m)| {
                self.iter()
                    .skip(i)
                    .step_by(n_item)
                    .skip(n_total - n_sample.unwrap_or(n_total))
                    .map(|&x| x - m)
                    .fold(0f64, |a, x| a + x * x)
                    / n_sample.unwrap_or(n_total) as f64
            })
            .collect()
    }
    /// Returns the standard deviation values
    ///
    /// Optionally, the statistical moment is evaluated on the last `n_sample`
    fn std(&self, n_sample: Option<usize>) -> Vec<f64>
    where
        Self: Deref<Target = Vec<f64>> + OpticalMetrics,
    {
        self.var(n_sample).iter().map(|x| x.sqrt()).collect()
    }
}
impl Stats for TipTilt {}
impl Stats for SegmentTipTilt {}
impl Stats for SegmentPiston {}

#[cfg(test)]
mod tests {
    use super::*;
    use skyangle::Conversion;
    use std::iter::once;

    #[test]
    fn tiptilt() {
        let mut m1_rbm = vec![vec![0f64; 6]; 7];
        let mut m2_rbm = vec![vec![0f64; 6]; 7];
        m1_rbm
            .iter_mut()
            .step_by(2)
            .for_each(|str| str[3] = 1f64.from_arcsec());
        m2_rbm
            .iter_mut()
            .skip(1)
            .step_by(2)
            .take(3)
            .for_each(|str| str[3] = 1f64.from_arcsec());
        m1_rbm[6][3] = 0.5f64.from_arcsec();
        m2_rbm[6][3] = (-4f64).from_arcsec();
        let lom = LOM::builder()
            .into_iter_rigid_body_motions(once((m1_rbm, m2_rbm)))
            .build()
            .unwrap();
        let stt = lom.segment_tiptilt();
        let mag: Vec<_> = stt[..7]
            .iter()
            .zip(&stt[7..])
            .map(|(x, y)| x.hypot(*y).to_arcsec())
            .collect();
        print!("Segment tiptilt : {:.3?} mas", mag);
    }

    #[test]
    fn segment_wfe_rms() {
        let mut m1_rbm = vec![vec![0f64; 6]; 7];
        let mut m2_rbm = vec![vec![0f64; 6]; 7];
        for i in 0..6 {
            m1_rbm[i][2] = 100e-9 * (i + 1) as f64;
        }
        m2_rbm[6][2] = -100e-9;
        let lom = LOM::builder()
            .into_iter_rigid_body_motions(once((m1_rbm, m2_rbm)))
            .build()
            .unwrap();
        let swferms = lom.segment_wfe_rms::<-9>();
        print!("Segment WFE RMS : {:.0?} mas", swferms);
    }
}
