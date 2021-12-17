//! # Giant Magellan Telescope Linear Optical Model
//!
//! Optical linear transformations applied to the rigid body motions of the primary and secondary segmented mirrors of the GMT
//!
//! The optical sensitivities can be downloaded from [here](https://s3.us-west-2.amazonaws.com/gmto.modeling/optical_sensitivities.rs.bin),
//! or they can be recomputed with the [makesens](../makesens/index.html) binary compiled with the `crseo` features and run on computer with a NVIDIA GPU.

use bincode;
use std::{
    fs::File,
    marker::PhantomData,
    path::{Path, PathBuf},
};

pub mod optical_sensitivities;
pub use optical_sensitivities::OpticalSensitivities;
pub mod rigid_body_motions;
pub use rigid_body_motions::RigidBodyMotions;

#[derive(thiserror::Error, Debug)]
pub enum OpticalSensitivitiesError {
    #[error("sensitivities file not found (optical_sensitivities.rs.bin)")]
    SensitivityFile(#[from] std::io::Error),
    #[error("sensitivities cannot be loaded from optical_sensitivities.rs.bin")]
    SensitivityData(#[from] bincode::Error),
    #[error("segment tip-tilt sensitivity is missing")]
    SegmentTipTilt,
}
type Result<T> = std::result::Result<T, OpticalSensitivitiesError>;

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
        <Vec<OpticalSensitivities> as Bin>::load(self.path.join(self.filename))
    }
}
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
impl LoaderTrait<RigidBodyMotions> for Loader<RigidBodyMotions> {
    /// Loads M1 and M2 rigid body motions
    fn load(self) -> Result<RigidBodyMotions> {
        RigidBodyMotions::from_parquet(self.path.join(self.filename))
    }
}

/// LOM builder
#[derive(Default)]
pub struct LOMBuilder {
    sens_loader: Loader<Vec<OpticalSensitivities>>,
    rbm_loader: Loader<RigidBodyMotions>,
}
impl LOMBuilder {
    /// Sets the loader for a [Vec] of [OpticalSensitivities]
    pub fn optical_sensitivities(self, sens_loader: Loader<Vec<OpticalSensitivities>>) -> Self {
        Self {
            sens_loader,
            ..self
        }
    }
    /// Sets the loader for [RigidBodyMotions]
    pub fn rigid_body_motions(self, rbm_loader: Loader<RigidBodyMotions>) -> Self {
        Self { rbm_loader, ..self }
    }
    /// Creates a [LOM]
    pub fn build(self) -> Result<LOM> {
        Ok(LOM {
            sens: self.sens_loader.load()?,
            rbm: self.rbm_loader.load()?,
        })
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
    /// Returns the pupil average tip and tilt
    ///
    /// The tip-tilt vector is given as `[x1,y1,...,xi,yi,...,xn,yn]` where i is the time index
    pub fn tiptilt(&self) -> Vec<f64> {
        self.sens.as_slice()[OpticalSensitivities::TipTilt(vec![])].into_optics(&self.rbm.data)
    }
    /// Returns the segment piston in the telescope exit pupil
    ///
    /// The segment piston vector is given as `[p11,p21,...,p71,...,p1i,p2i,...,p7i,...,p1n,p2n,...,p7n]` where i is the time index
    pub fn piston(&self) -> Vec<f64> {
        self.sens.as_slice()[OpticalSensitivities::SegmentPiston(vec![])]
            .into_optics(&self.rbm.data)
    }
    /// Returns the segment averaged tip and tilt in the telescope exit pupil
    ///
    /// The segment tip-tilt vector is given as `[x11,x21,...,x71,y11,y21,...,y71,...,x1i,x2i,...,x7i,y1i,y2i,...,y7i,...,x1n,x2n,...,x7n,y1n,y2n,...,y7n]` where i is the time index
    pub fn segment_tiptilt(&self) -> Vec<f64> {
        self.sens.as_slice()[OpticalSensitivities::SegmentTipTilt(vec![])]
            .into_optics(&self.rbm.data)
    }
}
