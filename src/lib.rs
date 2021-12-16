use bincode;
use std::{
    fs::File,
    marker::PhantomData,
    path::{Path, PathBuf},
};

pub mod optical_sensitivities;
use optical_sensitivities::OpticalSensitivities;
pub mod rigid_body_motions;
use rigid_body_motions::RigidBodyMotions;

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

/// Sensitivities serialization into a bincode file
pub trait Bin {
    fn dump<P: AsRef<Path>>(self, path: P) -> Result<Self>
    where
        Self: Sized;
    fn load<P: AsRef<Path>>(path: P) -> Result<Self>
    where
        Self: Sized;
}
impl Bin for Vec<OpticalSensitivities> {
    /// Serializes sensitivities
    ///
    /// Saves sensitivities to 'path'
    fn dump<P: AsRef<Path>>(self, path: P) -> Result<Self> {
        bincode::serialize_into(File::create(path)?, &self)?;
        Ok(self)
    }
    fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        Ok(bincode::deserialize_from(File::open(path)?)?)
    }
}

pub struct Loader<T> {
    path: PathBuf,
    filename: String,
    phantom: PhantomData<T>,
}
pub trait LoaderTrait<T> {
    fn load(self) -> Result<T>;
}
impl<T> Loader<T> {
    pub fn path<P: AsRef<Path>>(self, path: P) -> Self {
        Self {
            path: Path::new(path.as_ref()).to_path_buf(),
            ..self
        }
    }
    pub fn filename(self, filename: &str) -> Self {
        Self {
            filename: String::from(filename),
            ..self
        }
    }
}
impl Default for Loader<Vec<OpticalSensitivities>> {
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
    ///
    /// Look in the current directory for the file: "optical_sensitivities.rs.bin"
    fn load(self) -> Result<Vec<OpticalSensitivities>> {
        <Vec<OpticalSensitivities> as Bin>::load(self.path.join(self.filename))
    }
}
impl Default for Loader<RigidBodyMotions> {
    fn default() -> Self {
        Self {
            path: Path::new(".").to_path_buf(),
            filename: String::from("data.parquet"),
            phantom: PhantomData,
        }
    }
}
impl LoaderTrait<RigidBodyMotions> for Loader<RigidBodyMotions> {
    fn load(self) -> Result<RigidBodyMotions> {
        RigidBodyMotions::from_parquet(self.path.join(self.filename))
    }
}

#[derive(Default)]
pub struct LOMBuilder {
    sens_loader: Loader<Vec<OpticalSensitivities>>,
    rbm_loader: Loader<RigidBodyMotions>,
}
impl LOMBuilder {
    pub fn optical_sensitivities(self, sens_loader: Loader<Vec<OpticalSensitivities>>) -> Self {
        Self {
            sens_loader,
            ..self
        }
    }
    pub fn rigid_body_motions(self, rbm_loader: Loader<RigidBodyMotions>) -> Self {
        Self { rbm_loader, ..self }
    }
    pub fn build(self) -> Result<LOM> {
        Ok(LOM {
            sens: self.sens_loader.load()?,
            rbm: self.rbm_loader.load()?,
        })
    }
}

pub struct LOM {
    sens: Vec<OpticalSensitivities>,
    rbm: RigidBodyMotions,
}
impl LOM {
    pub fn builder() -> LOMBuilder {
        Default::default()
    }
    pub fn tiptilt(&self) -> Vec<f64> {
        self.sens.as_slice()[OpticalSensitivities::TipTilt(vec![])].into_optics(&self.rbm.data)
    }
    pub fn piston(&self) -> Vec<f64> {
        self.sens.as_slice()[OpticalSensitivities::SegmentPiston(vec![])]
            .into_optics(&self.rbm.data)
    }
    pub fn segment_tiptilt(&self) -> Vec<f64> {
        self.sens.as_slice()[OpticalSensitivities::SegmentTipTilt(vec![])]
            .into_optics(&self.rbm.data)
    }
}
