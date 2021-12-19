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
use std::{
    fs::File,
    marker::PhantomData,
    ops::Deref,
    path::{Path, PathBuf},
};

pub mod optical_sensitivities;
pub use optical_sensitivities::OpticalSensitivities;
pub mod rigid_body_motions;
pub use rigid_body_motions::RigidBodyMotions;

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
}
type Result<T> = std::result::Result<T, LinearOpticalModelError>;

/// Sensitivities serialization into a [bincode] file
pub trait Bin {
    fn dump<P: AsRef<Path>>(self, path: P) -> Result<Self>
    where
        Self: Sized;
    fn load<P: AsRef<Path>>(path: P) -> Result<Self>
    where
        Self: Sized;
}
impl Bin for Vec<OpticalSensitivities> {
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
impl Default for Loader<Vec<OpticalSensitivities>> {
    /// Default [Loader] for [Vec] of [OpticalSensitivities],
    /// expecting the file `optical_sensitivities.rs.bin` in the current folder
    fn default() -> Self {
        Self {
            path: Path::new(".").to_path_buf(),
            filename: String::from("optical_sensitivities.rs.bin"),
            phantom: PhantomData,
        }
    }
}
impl LoaderTrait<Vec<OpticalSensitivities>> for Loader<Vec<OpticalSensitivities>> {
    /// Loads precomputed optical sensitivities
    fn load(self) -> Result<Vec<OpticalSensitivities>> {
        println!("Loading optical sensitivities ...");
        <Vec<OpticalSensitivities> as Bin>::load(self.path.join(self.filename))
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
    sens: Option<Vec<OpticalSensitivities>>,
    rbm: Option<RigidBodyMotions>,
}
impl LOMBuilder {
    /// Sets the [bincode] loader for a [Vec] of [OpticalSensitivities]
    pub fn load_optical_sensitivities(
        self,
        sens_loader: Loader<Vec<OpticalSensitivities>>,
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
            rbm: self
                .rbm
                .ok_or(LinearOpticalModelError::MissingRigidBodyMotions)?,
        })
    }
}

pub struct TipTilt(Vec<f64>);
impl Deref for TipTilt {
    type Target = Vec<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
pub struct SegmentTipTilt(Vec<f64>);
impl Deref for SegmentTipTilt {
    type Target = Vec<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
pub struct SegmentPiston(Vec<f64>);
impl Deref for SegmentPiston {
    type Target = Vec<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
pub trait Stats {
    fn mean(&self, n_sample: Option<usize>) -> Vec<f64>;
    fn var(&self, n_sample: Option<usize>) -> Vec<f64>;
    fn std(&self, n_sample: Option<usize>) -> Vec<f64> {
        self.var(n_sample).iter().map(|x| x.sqrt()).collect()
    }
}
impl Stats for TipTilt {
    fn mean(&self, n_sample: Option<usize>) -> Vec<f64> {
        let n_total = self.len() / 2;
        vec![
            self.iter()
                .step_by(2)
                .skip(n_total - n_sample.unwrap_or(n_total))
                .sum::<f64>()
                / n_sample.unwrap_or(n_total) as f64,
            self.iter()
                .skip(1)
                .step_by(2)
                .skip(n_total - n_sample.unwrap_or(n_total))
                .sum::<f64>()
                / n_sample.unwrap_or(n_total) as f64,
        ]
    }
    fn var(&self, n_sample: Option<usize>) -> Vec<f64> {
        let mean = self.mean(n_sample);
        let n_total = self.len() / 2;
        vec![
            self.iter()
                .step_by(2)
                .skip(n_total - n_sample.unwrap_or(n_total))
                .map(|&x| x - mean[0])
                .fold(0f64, |a, x| a + x * x)
                / n_sample.unwrap_or(n_total) as f64,
            self.iter()
                .skip(1)
                .step_by(2)
                .skip(n_total - n_sample.unwrap_or(n_total))
                .map(|&x| x - mean[1])
                .fold(0f64, |a, x| a + x * x)
                / n_sample.unwrap_or(n_total) as f64,
        ]
    }
}

/// Linear Optical Model
pub struct LOM {
    sens: Vec<OpticalSensitivities>,
    rbm: RigidBodyMotions,
}
impl LOM {
    /// Returns the [builder](LOMBuilder)
    pub fn builder() -> LOMBuilder {
        Default::default()
    }
    /// Returns a the time vector
    pub fn time(&self) -> Vec<f64> {
        self.rbm.time()
    }
    /// Returns the pupil average tip and tilt in `[mas]`
    ///
    /// The tip-tilt vector is given as `[x1,y1,...,xi,yi,...,xn,yn]` where i is the time index
    pub fn tiptilt(&self) -> TipTilt {
        TipTilt(
            self.sens.as_slice()[OpticalSensitivities::TipTilt(vec![])]
                .into_optics(self.rbm.data()),
        )
    }
    /// Returns the segment piston in the telescope exit pupil in `[nm]`
    ///
    /// The segment piston vector is given as `[p11,p21,...,p71,...,p1i,p2i,...,p7i,...,p1n,p2n,...,p7n]` where i is the time index
    pub fn segment_piston(&self) -> SegmentPiston {
        SegmentPiston(
            self.sens.as_slice()[OpticalSensitivities::SegmentPiston(vec![])]
                .into_optics(self.rbm.data()),
        )
    }
    /// Returns the segment averaged tip and tilt in the telescope exit pupil in `[mas]`
    ///
    /// The segment tip-tilt vector is given as `[x11,x21,...,x71,y11,y21,...,y71,...,x1i,x2i,...,x7i,y1i,y2i,...,y7i,...,x1n,x2n,...,x7n,y1n,y2n,...,y7n]` where i is the time index
    pub fn segment_tiptilt(&self) -> SegmentTipTilt {
        SegmentTipTilt(
            self.sens.as_slice()[OpticalSensitivities::SegmentTipTilt(vec![])]
                .into_optics(self.rbm.data()),
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
