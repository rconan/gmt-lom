use std::fmt::Display;

use skyangle::Conversion;

use crate::{
    Formatting, LinearOpticalModelError, Loader, LoaderTrait, OpticalSensitivities,
    OpticalSensitivity, RigidBodyMotions, SegmentPiston, SegmentTipTilt, TipTilt,
};

type Result<T> = std::result::Result<T, LinearOpticalModelError>;

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
    pub fn table_rigid_body_motions(
        self,
        table: &crate::Table,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<Self> {
        Ok(Self {
            rbm: Some(RigidBodyMotions::from_table(
                table,
                m1_rbm_label,
                m2_rbm_label,
            )?),
            ..self
        })
    }
    #[cfg(feature = "apache")]
    pub fn rigid_body_motions_record(
        self,
        record: &arrow::record_batch::RecordBatch,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<Self> {
        Ok(Self {
            rbm: Some(RigidBodyMotions::from_record(
                record,
                m1_rbm_label,
                m2_rbm_label,
            )?),
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

/// Linear Optical Model
#[derive(Debug, Clone)]
pub struct LOM {
    sens: OpticalSensitivities,
    pub rbm: RigidBodyMotions,
}
impl Display for LOM {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.rbm.fmt(f)
    }
}
impl<'a> TryFrom<&'a [u8]> for LOM {
    type Error = LinearOpticalModelError;

    fn try_from(bytes: &'a [u8]) -> std::result::Result<Self, Self::Error> {
        Ok(Self {
            sens: bincode::deserialize(bytes)?,
            rbm: Default::default(),
        })
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
        TipTilt(self.sens[OpticalSensitivity::<84>::TipTilt(vec![])].into_optics(self.rbm.data()))
    }
    pub fn tiptilt_mas(&self) -> TipTilt {
        TipTilt(
            self.sens[OpticalSensitivity::<84>::TipTilt(vec![])]
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
            self.sens[OpticalSensitivity::<84>::SegmentPiston(vec![])].into_optics(self.rbm.data()),
        )
    }
    /// Returns the segment averaged tip and tilt in the telescope exit pupil in `[rd]`
    ///
    /// The segment tip-tilt vector is given as `[x11,x21,...,x71,y11,y21,...,y71,...,x1i,x2i,...,x7i,y1i,y2i,...,y7i,...,x1n,x2n,...,x7n,y1n,y2n,...,y7n]` where i is the time index
    pub fn segment_tiptilt(&self) -> SegmentTipTilt {
        SegmentTipTilt(
            self.sens[OpticalSensitivity::<84>::SegmentTipTilt(vec![])]
                .into_optics(self.rbm.data()),
        )
    }
    pub fn segment_tiptilt_mas(&self) -> SegmentTipTilt {
        SegmentTipTilt(
            self.sens[OpticalSensitivity::<84>::SegmentTipTilt(vec![])]
                .into_optics(self.rbm.data())
                .into_iter()
                .map(|x| x.to_mas())
                .collect::<Vec<f64>>(),
        )
    }
    /// Returns the wavefront within the exit pupil in `[m]`
    pub fn masked_wavefront(&self) -> Vec<f64> {
        self.sens[OpticalSensitivity::<84>::Wavefront(vec![])].into_optics(self.rbm.data())
    }
    /// Returns the wavefront of each segment within the exit pupil in `[m]`
    pub fn segment_wavefront(&self) -> Vec<Vec<f64>> {
        let mut wavefront = self.sens[OpticalSensitivity::<84>::Wavefront(vec![])]
            .into_optics(self.rbm.data())
            .into_iter();
        if let OpticalSensitivity::SegmentMask(mask) =
            &self.sens[OpticalSensitivity::<84>::SegmentMask(vec![])]
        {
            (1..=7)
                .map(|sid| {
                    wavefront
                        .by_ref()
                        .zip(mask)
                        .filter_map(|(w, m)| (*m == sid).then_some(w))
                        .collect::<Vec<f64>>()
                })
                .collect()
        } else {
            panic!("`SegmentMask` is missing from `OpticalSensitivities`")
        }
    }
    /// Returns the WFE RMS of each segment within the exit pupil in `[m]`
    pub fn segment_wfe_rms<const E: i32>(&self) -> Vec<f64> {
        let wavefront = self.masked_wavefront();
        let wavefront_iter = wavefront.as_slice();
        if let OpticalSensitivity::SegmentMask(mask) =
            &self.sens[OpticalSensitivity::<84>::SegmentMask(vec![])]
        {
            (1..=7)
                .map(|sid| {
                    let (i, s) = wavefront_iter
                        .iter()
                        .zip(mask)
                        .filter_map(|(w, m)| (*m == sid).then_some(w))
                        .enumerate()
                        .fold((0usize, 0f64), |(_, mut s), (i, w)| {
                            s += w * w;
                            (i, s)
                        });
                    let n = (i + 1) as f64;
                    let mut wfe_rms = (s / n).sqrt();
                    if E != 0 {
                        wfe_rms *= 10f64.powi(-E);
                    }
                    wfe_rms
                })
                .collect()
        } else {
            panic!("`SegmentMask` is missing from `OpticalSensitivities`")
        }
    }
    /// Returns the wavefront in the exit pupil in `[rmm]`
    pub fn wavefront(&self) -> Vec<f64> {
        let mut wavefront = self.sens[OpticalSensitivity::<84>::Wavefront(vec![])]
            .into_optics(self.rbm.data())
            .into_iter();
        if let OpticalSensitivity::PupilMask(mask) =
            &self.sens[OpticalSensitivity::<84>::PupilMask(vec![])]
        {
            mask.into_iter()
                .map(|&mask| {
                    if mask {
                        wavefront.next().unwrap()
                    } else {
                        0f64
                    }
                })
                .collect()
        } else {
            panic!("`PupilMask` is missing from `OpticalSensitivities`")
        }
    }
}
