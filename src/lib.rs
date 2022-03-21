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
use skyangle::Conversion;
use std::{
    env,
    fmt::Display,
    fs::File,
    marker::PhantomData,
    ops::{Deref, DerefMut},
    path::{Path, PathBuf},
    slice::Chunks,
};

mod optical_sensitivities;
pub use optical_sensitivities::{from_opticals, OpticalSensitivities, OpticalSensitivity};
mod rigid_body_motions;
pub use rigid_body_motions::RigidBodyMotions;
#[cfg(feature = "apache")]
mod pmts;
#[cfg(feature = "apache")]
pub use pmts::Pmt;
#[cfg(feature = "apache")]
mod table;
#[cfg(feature = "apache")]
pub use table::Table;

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
impl Bin for OpticalSensitivities {
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
impl Default for Loader<OpticalSensitivities> {
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
impl LoaderTrait<OpticalSensitivities> for Loader<OpticalSensitivities> {
    /// Loads precomputed optical sensitivities
    fn load(self) -> Result<OpticalSensitivities> {
        println!("Loading optical sensitivities ...");
        <OpticalSensitivities as Bin>::load(self.path.join(self.filename))
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
        RigidBodyMotions::from_parquet(self.path.join(self.filename))
    }
}

/// LOM builder
#[derive(Default)]
pub struct LOMBuilder {
    sens: Option<OpticalSensitivities>,
    rbm: Option<RigidBodyMotions>,
}
impl LOMBuilder {
    /// Sets the [bincode] loader for a [Vec] of [OpticalSensitivity]
    pub fn load_optical_sensitivities(
        self,
        sens_loader: Loader<OpticalSensitivities>,
    ) -> Result<Self> {
        Ok(Self {
            sens: Some(sens_loader.load()?),
            ..self
        })
    }
    /// Sets the [parquet](https://docs.rs/parquet) loader for [RigidBodyMotions]
    #[cfg(feature = "apache")]
    pub fn load_rigid_body_motions(self, rbm_loader: Loader<RigidBodyMotions>) -> Result<Self> {
        Ok(Self {
            rbm: Some(rbm_loader.load()?),
            ..self
        })
    }
    #[cfg(feature = "apache")]
    pub fn table_rigid_body_motions(self, table: &Table) -> Result<Self> {
        Ok(Self {
            rbm: Some(RigidBodyMotions::from_table(table)?),
            ..self
        })
    }
    /// Sets [RigidBodyMotions] from an iterator of [tuple] of M1 and M2 segments [Vec] of 6 rigid body motions (Txyz and Rxyz)
    pub fn into_iter_rigid_body_motions(
        self,
        data: impl Iterator<Item = (Vec<Vec<f64>>, Vec<Vec<f64>>)>,
    ) -> Self {
        Self {
            rbm: Some(data.collect()),
            ..self
        }
    }
    /// Sets [RigidBodyMotions] from an iterator of [tuple] of M1 and M2 [slice] of 42 rigid body motions
    pub fn iter_rigid_body_motions<'a>(
        self,
        data: impl Iterator<Item = (&'a [f64], &'a [f64])>,
    ) -> Self {
        Self {
            rbm: Some(data.collect()),
            ..self
        }
    }
    /// Creates a [LOM]
    pub fn build(self) -> Result<LOM> {
        Ok(LOM {
            sens: self.sens.unwrap_or(Loader::default().load()?),
            rbm: self.rbm.unwrap_or_default(),
        })
    }
}

/// Type holding the tip-tilt values
#[derive(Serialize)]
pub struct TipTilt(Vec<f64>);
/// Type holding the segment tip-tilt values
#[derive(Serialize)]
pub struct SegmentTipTilt(Vec<f64>);
/// Type holding the segment piston values
#[derive(Serialize)]
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

/// Linear Optical Model
pub struct LOM {
    sens: OpticalSensitivities,
    pub rbm: RigidBodyMotions,
}
impl Display for LOM {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.rbm.fmt(f)
    }
}
impl LOM {
    pub fn latex(&mut self) {
        self.rbm.format = Formatting::Latex;
    }
    pub fn adhoc(&mut self) {
        self.rbm.format = Formatting::AdHoc;
    }
    /// Returns the [builder](LOMBuilder)
    pub fn builder() -> LOMBuilder {
        Default::default()
    }
    /// Returns the number of rigid body motions sample `n`
    pub fn len(&self) -> usize {
        self.rbm.len()
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Returns a the time vector
    pub fn time(&self) -> Vec<f64> {
        self.rbm.time()
    }
    /// Returns the pupil average tip and tilt in `[rd]`
    ///
    /// The tip-tilt vector is given as `[x1,y1,...,xi,yi,...,xn,yn]` where i is the time index
    pub fn tiptilt(&self) -> TipTilt {
        TipTilt(self.sens[OpticalSensitivity::TipTilt(vec![])].into_optics(self.rbm.data()))
    }
    pub fn tiptilt_mas(&self) -> TipTilt {
        TipTilt(
            self.sens[OpticalSensitivity::TipTilt(vec![])]
                .into_optics(self.rbm.data())
                .into_iter()
                .map(|x| x.to_mas())
                .collect::<Vec<f64>>(),
        )
    }
    /// Returns the segment piston in the telescope exit pupil in `[m]`
    ///
    /// The segment piston vector is given as `[p11,p21,...,p71,...,p1i,p2i,...,p7i,...,p1n,p2n,...,p7n]` where i is the time index
    pub fn segment_piston(&self) -> SegmentPiston {
        SegmentPiston(
            self.sens[OpticalSensitivity::SegmentPiston(vec![])].into_optics(self.rbm.data()),
        )
    }
    /// Returns the segment averaged tip and tilt in the telescope exit pupil in `[rd]`
    ///
    /// The segment tip-tilt vector is given as `[x11,x21,...,x71,y11,y21,...,y71,...,x1i,x2i,...,x7i,y1i,y2i,...,y7i,...,x1n,x2n,...,x7n,y1n,y2n,...,y7n]` where i is the time index
    pub fn segment_tiptilt(&self) -> SegmentTipTilt {
        SegmentTipTilt(
            self.sens[OpticalSensitivity::SegmentTipTilt(vec![])].into_optics(self.rbm.data()),
        )
    }
    pub fn segment_tiptilt_mas(&self) -> SegmentTipTilt {
        SegmentTipTilt(
            self.sens[OpticalSensitivity::SegmentTipTilt(vec![])]
                .into_optics(self.rbm.data())
                .into_iter()
                .map(|x| x.to_mas())
                .collect::<Vec<f64>>(),
        )
    }
}

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
            .map(|(x, y)| x.hypot(*y))
            .collect();
        print!("Segment tiptilt : {:.0?} mas", mag);
    }
}
