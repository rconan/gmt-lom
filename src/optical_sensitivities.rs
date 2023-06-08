use std::ops::Deref;

use crate::{LinearOpticalModelError, Result};
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Optical sensitivities
///
/// Linear transformation of M1 and M2 rigid body motions into wavefront and wavefront piston and tip-tilt modes
#[derive(Serialize, Deserialize, Clone)]
pub struct OpticalSensitivities(Vec<OpticalSensitivity>);
impl Deref for OpticalSensitivities {
    type Target = [OpticalSensitivity];
    fn deref(&self) -> &Self::Target {
        self.0.as_slice()
    }
}
/// Optical sensitivity
///
/// Linear transformation of M1 and M2 rigid body motions into wavefront and wavefront piston and tip-tilt modes
#[derive(Serialize, Deserialize, Clone)]
pub enum OpticalSensitivity {
    /// Wavefront sensitivity `[nx84]` where n in the pupil resolution
    Wavefront(Vec<f64>),
    /// Exit pupil tip-tilt sensitivity `[2x84]`
    TipTilt(Vec<f64>),
    /// Exit pupil segment tip-tilt `[14x84]`
    SegmentTipTilt(Vec<f64>),
    /// Exit pupil segment piston `[7x84]`
    SegmentPiston(Vec<f64>),
    SegmentMask(Vec<i32>),
    PupilMask(Vec<bool>),
}
impl<'a> From<&'a OpticalSensitivity> for na::DMatrix<f64> {
    fn from(sens: &'a OpticalSensitivity) -> Self {
        use OpticalSensitivity::*;
        match sens {
            Wavefront(val) => Some(na::DMatrix::from_column_slice(val.len() / 84, 84, val)),
            TipTilt(val) => Some(na::DMatrix::from_column_slice(2, 84, val)),
            SegmentTipTilt(val) => Some(na::DMatrix::from_column_slice(14, 84, val)),
            SegmentPiston(val) => Some(na::DMatrix::from_column_slice(7, 84, val)),
            _ => None,
        }
        .unwrap()
    }
}
impl PartialEq<OpticalSensitivity> for OpticalSensitivity {
    fn eq(&self, other: &OpticalSensitivity) -> bool {
        use OpticalSensitivity::*;
        match (self, other) {
            (Wavefront(_), Wavefront(_)) => true,
            (TipTilt(_), TipTilt(_)) => true,
            (SegmentTipTilt(_), SegmentTipTilt(_)) => true,
            (SegmentPiston(_), SegmentPiston(_)) => true,
            (SegmentMask(_), SegmentMask(_)) => true,
            _ => false,
        }
    }
}
impl std::ops::Index<OpticalSensitivity> for OpticalSensitivities {
    type Output = OpticalSensitivity;

    fn index(&self, index: OpticalSensitivity) -> &Self::Output {
        self.iter()
            .find_map(|s| if index == *s { Some(s) } else { None })
            .unwrap()
    }
}
impl<'a> From<&'a OpticalSensitivity> for &'a [f64] {
    fn from(sens: &'a OpticalSensitivity) -> Self {
        use OpticalSensitivity::*;
        match sens {
            Wavefront(val) | TipTilt(val) | SegmentTipTilt(val) | SegmentPiston(val) => {
                Some(val.as_slice())
            }
            _ => None,
        }
        .unwrap()
    }
}

impl OpticalSensitivity {
    /// Returns M1 wavefront sensitivities `[nx42]`
    pub fn m1_wavefront(&self) -> Result<na::DMatrix<f64>> {
        match self {
            OpticalSensitivity::Wavefront(sens) => {
                let n = sens.len() / 84;
                let (_, m1_tr) = sens.split_at(n * 42);
                Ok(na::DMatrix::from_iterator(
                    n,
                    42,
                    m1_tr.chunks(n).flat_map(|x| x.to_vec()),
                ))
            }
            _ => Err(LinearOpticalModelError::SegmentTipTilt),
        }
    }
    /// Returns M2 segment tip-tilt sensitivities `[14x14]`
    pub fn m2_rxy(&self) -> Result<na::DMatrix<f64>> {
        match self {
            OpticalSensitivity::SegmentTipTilt(sens) => {
                let (_, m2_tr) = sens.split_at(14 * 42);
                Ok(na::DMatrix::from_iterator(
                    14,
                    14,
                    m2_tr
                        .chunks(14 * 3)
                        .skip(1)
                        .step_by(2)
                        .flat_map(|x| (&x[..14 * 2]).to_vec()),
                ))
            }
            _ => Err(LinearOpticalModelError::SegmentTipTilt),
        }
    }
    pub fn into_optics(&self, rbm: &na::DMatrix<f64>) -> Vec<f64> {
        match self {
            /*OpticalSensitivity::Wavefront(sens) => {
                let n = sens.len() / 84;
                //println!("n: {}", n);
                let sensitivity = na::DMatrix::from_column_slice(n, 84, sens);
                //let now = Instant::now();
                let wfe_var = {
                    let n_buf = 1_000;
                    let mut buf = na::DMatrix::<f64>::zeros(n, n_buf);
                    let mut s = 0;
                    let mut var = 0f64;
                    loop {
                        if s + n_buf > n_sample {
                            s -= n_buf;
                            let n_last = n_sample - s;
                            let mut buf = na::DMatrix::<f64>::zeros(n, n_last);
                            buf.gemm(1f64, &sensitivity, &rbm.columns(s, n_last), 0f64);
                            var += buf.row_variance().as_slice().into_iter().sum::<f64>();
                            break var;
                        } else {
                            buf.gemm(1f64, &sensitivity, &rbm.columns(s, n_buf), 0f64);
                            var += buf.row_variance().as_slice().into_iter().sum::<f64>();
                        }
                        s += n_buf;
                    }
                };
                let value = 1e9 * (wfe_var / n_sample as f64).sqrt();
                OpticalWindLoad::Wavefront(value)
                /*println!(
                    "Wavefront: {:6.0}nm in {:.3}s", value,
                    now.elapsed().as_secs_f64()
                );*/
            }*/
            OpticalSensitivity::TipTilt(sens) => {
                let sensitivity = na::DMatrix::from_column_slice(2, 84, sens);
                let tip_tilt = sensitivity * rbm;
                tip_tilt.as_slice().to_owned()
            }
            OpticalSensitivity::SegmentTipTilt(sens) => {
                let sensitivity = na::DMatrix::from_column_slice(14, 84, sens);
                let segment_tip_tilt = sensitivity * rbm;
                segment_tip_tilt.as_slice().to_owned()
            }
            OpticalSensitivity::SegmentPiston(sens) => {
                let sensitivity = na::DMatrix::from_column_slice(7, 84, sens);
                let segment_piston = sensitivity * rbm;
                let mut v: Vec<f64> = vec![];
                for (k, row) in segment_piston.row_iter().take(6).enumerate() {
                    //println!("{}: {:?}", k, row.shape());
                    v.extend(
                        &mut segment_piston
                            .rows(k + 1, 6 - k)
                            .row_iter()
                            .flat_map(|y| (y - row).as_slice().to_owned()),
                    );
                }
                segment_piston.as_slice().to_owned()
            }
            _ => unimplemented!(),
        }
    }
    /*
       pub fn transform(&self, optics_model: &WindLoadedGmtInner) -> OpticalWindLoad {
           let n_sample = optics_model.n_sample;
           let rbm = &optics_model.rbm;
           match self {
               OpticalSensitivity::Wavefront(sens) => {
                   let n = sens.len() / 84;
                   //println!("n: {}", n);
                   let sensitivity = na::DMatrix::from_column_slice(n, 84, sens);
                   //let now = Instant::now();
                   let wfe_var = {
                       let n_buf = 1_000;
                       let mut buf = na::DMatrix::<f64>::zeros(n, n_buf);
                       let mut s = 0;
                       let mut var = 0f64;
                       loop {
                           if s + n_buf > n_sample {
                               s -= n_buf;
                               let n_last = n_sample - s;
                               let mut buf = na::DMatrix::<f64>::zeros(n, n_last);
                               buf.gemm(1f64, &sensitivity, &rbm.columns(s, n_last), 0f64);
                               var += buf.row_variance().as_slice().into_iter().sum::<f64>();
                               break var;
                           } else {
                               buf.gemm(1f64, &sensitivity, &rbm.columns(s, n_buf), 0f64);
                               var += buf.row_variance().as_slice().into_iter().sum::<f64>();
                           }
                           s += n_buf;
                       }
                   };
                   let value = 1e9 * (wfe_var / n_sample as f64).sqrt();
                   OpticalWindLoad::Wavefront(value)
                   /*println!(
                       "Wavefront: {:6.0}nm in {:.3}s", value,
                       now.elapsed().as_secs_f64()
                   );*/
               }
               OpticalSensitivity::TipTilt(sens) => {
                   let sensitivity = na::DMatrix::from_column_slice(2, 84, sens);
                   let tip_tilt = (sensitivity * rbm).map(|x| x.to_mas());
                   let values = tip_tilt
                       .column_variance()
                       .map(|x| x.sqrt())
                       .as_slice()
                       .to_owned();
                   //println!("TT: {:2.0?}mas", &values);
                   OpticalWindLoad::TipTilt(values)
               }
               OpticalSensitivity::SegmentTipTilt(sens) => {
                   let sensitivity = na::DMatrix::from_column_slice(14, 84, sens);
                   let segment_tip_tilt = (sensitivity * rbm).map(|x| x.to_mas());
                   let values: Vec<_> = segment_tip_tilt
                       .column_variance()
                       .map(|x| x.sqrt())
                       .as_slice()
                       .chunks(7)
                       .map(|x| x.to_owned())
                       .collect();
                   //println!("Segment TT: {:2.0?}mas", values,);
                   OpticalWindLoad::SegmentTipTilt(values)
               }
               OpticalSensitivity::SegmentPiston(sens) => {
                   let sensitivity = na::DMatrix::from_column_slice(7, 84, sens);
                   let segment_piston = (sensitivity * rbm).map(|x| x * 1e9);
                   let mut v: Vec<f64> = vec![];
                   for (k, row) in segment_piston.row_iter().take(6).enumerate() {
                       //println!("{}: {:?}", k, row.shape());
                       v.extend(
                           &mut segment_piston
                               .rows(k + 1, 6 - k)
                               .row_iter()
                               .flat_map(|y| (y - row).as_slice().to_owned()),
                       );
                   }
                   let value = (na::DMatrix::from_vec(n_sample, 21, v)
                       .column_variance()
                       .sum()
                       / n_sample as f64)
                       .sqrt();
                   let values = segment_piston
                       .column_variance()
                       .map(|x| x.sqrt())
                       .as_slice()
                       .to_owned();
                   //println!("Diff. piston std: {:5.0}nm", value,);
                   //println!("Piston: {:3.0?}nm ; ", &values);
                   OpticalWindLoad::Piston([
                       PistonWindLoad::DifferentialSegmentPiston(value),
                       PistonWindLoad::SegmentPiston(values),
                   ])
               }
               OpticalSensitivity::SegmentMask(_) => OpticalWindLoad::WavefrontWoSegmentPiston(None),
           }
       }
    */
}
#[cfg(feature = "crseo")]
impl OpticalSensitivities {
    /// Computes optical sensitivities for M1 and M2 rigid body motions
    ///
    /// Returns a `Vec<OpticalSensitivity>` containing the linear transformations from M1 and M2 rigid body motions to
    /// wavefront, tip-tilt, segment tip-tilt and segment piston
    /// Optionally provides an optical model or uses: [`ceo!(GMT)`](crseo::GMT) and [`ceo!(SOURCE)`](crseo::SOURCE)
    pub fn compute(model: Option<(crseo::Gmt, crseo::Source)>) -> Result<Self> {
        use crseo::{Builder, FromBuilder, Gmt, Source};
        use skyangle::Conversion;
        println!("Computing optical sensitivities ...");
        let now = std::time::Instant::now();
        let (mut gmt, mut src) = model.unwrap_or((
            Gmt::builder().build().unwrap(),
            Source::builder().build().unwrap(),
        ));
        let stroke_fn = |dof| if dof < 3 { 1e-6 } else { 1f64.from_arcsec() };

        let mut tip_tilt = vec![];
        let mut segment_piston = vec![];
        let mut segment_tip_tilt = vec![];
        let mut phase = vec![];
        let n = (src.pupil_sampling * src.pupil_sampling) as usize;
        let mut amplitude = vec![true; n];
        for sid in 0..7 {
            for dof in 0..6 {
                let mut m1_rbm = vec![vec![0.; 6]; 7];
                let stroke = stroke_fn(dof);

                m1_rbm[sid][dof] = stroke;
                gmt.update(Some(&m1_rbm), None, None, None);

                src.through(&mut gmt).xpupil();
                amplitude
                    .iter_mut()
                    .zip(src.amplitude().into_iter())
                    .for_each(|(b, a)| {
                        *b = a > 0f32 && *b;
                    });
                let push_phase = src.phase().to_owned();
                let push_tip_tilt = src.gradients();
                let push_segment_piston = src.segment_piston();
                let push_segment_tip_tilt = src.segment_gradients();

                m1_rbm[sid][dof] = -stroke;
                gmt.update(Some(&m1_rbm), None, None, None);

                src.through(&mut gmt).xpupil();
                amplitude
                    .iter_mut()
                    .zip(src.amplitude().into_iter())
                    .for_each(|(b, a)| {
                        *b = a > 0f32 && *b;
                    });
                phase.extend(
                    src.phase()
                        .to_owned()
                        .into_iter()
                        .zip(push_phase.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
                tip_tilt.extend(
                    src.gradients()
                        .into_iter()
                        .zip(push_tip_tilt.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
                segment_piston.extend(
                    src.segment_piston()
                        .into_iter()
                        .zip(push_segment_piston.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
                segment_tip_tilt.extend(
                    src.segment_gradients()
                        .into_iter()
                        .zip(push_segment_tip_tilt.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
            }
        }
        for sid in 0..7 {
            for dof in 0..6 {
                let mut m2_rbm = vec![vec![0.; 6]; 7];
                let stroke = stroke_fn(dof);

                m2_rbm[sid][dof] = stroke;
                gmt.update(None, Some(&m2_rbm), None, None);

                src.through(&mut gmt).xpupil();
                amplitude
                    .iter_mut()
                    .zip(src.amplitude().into_iter())
                    .for_each(|(b, a)| {
                        *b = a > 0f32 && *b;
                    });
                let push_phase = src.phase().to_owned();
                let push_tip_tilt = src.gradients();
                let push_segment_piston = src.segment_piston();
                let push_segment_tip_tilt = src.segment_gradients();

                m2_rbm[sid][dof] = -stroke;
                gmt.update(None, Some(&m2_rbm), None, None);

                src.through(&mut gmt).xpupil();
                amplitude
                    .iter_mut()
                    .zip(src.amplitude().into_iter())
                    .for_each(|(b, a)| {
                        *b = a > 0f32 && *b;
                    });
                phase.extend(
                    src.phase()
                        .to_owned()
                        .into_iter()
                        .zip(push_phase.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
                tip_tilt.extend(
                    src.gradients()
                        .into_iter()
                        .zip(push_tip_tilt.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
                segment_piston.extend(
                    src.segment_piston()
                        .into_iter()
                        .zip(push_segment_piston.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
                segment_tip_tilt.extend(
                    src.segment_gradients()
                        .into_iter()
                        .zip(push_segment_tip_tilt.into_iter())
                        .map(|(l, r)| 0.5f64 * (r as f64 - l as f64) / stroke),
                );
            }
        }
        let optical_sensitivities = vec![
            OpticalSensitivity::Wavefront(
                phase
                    .chunks(n)
                    .flat_map(|pp| {
                        pp.iter()
                            .zip(amplitude.iter())
                            .filter(|(_, a)| **a)
                            .map(|(p, _)| *p)
                            .collect::<Vec<f64>>()
                    })
                    .collect(),
            ),
            OpticalSensitivity::TipTilt(tip_tilt),
            OpticalSensitivity::SegmentPiston(segment_piston),
            OpticalSensitivity::SegmentTipTilt(segment_tip_tilt),
            OpticalSensitivity::SegmentMask(
                src.segment_mask()
                    .iter()
                    .zip(amplitude.iter())
                    .filter(|(_, a)| **a)
                    .map(|(p, _)| *p)
                    .collect(),
            ),
            OpticalSensitivity::PupilMask(amplitude),
        ];
        println!(" ... done in {:.3}s", now.elapsed().as_secs_f64());
        Ok(Self(optical_sensitivities))
    }
}
pub fn from_opticals(senses: &[OpticalSensitivity]) -> na::DMatrix<f64> {
    let mats: Vec<na::DMatrix<f64>> = senses.iter().map(|s| s.into()).collect();
    let n_rows = mats.iter().map(|m| m.nrows()).sum::<usize>();
    let cols: Vec<_> = mats
        .iter()
        .flat_map(|m| {
            m.column_iter()
                .map(|c| na::DVector::from_column_slice(c.as_slice()))
                .collect::<Vec<na::DVector<f64>>>()
        })
        .collect();
    let data: Vec<_> = (0..84)
        .flat_map(|k| {
            (0..senses.len())
                .flat_map(|l| cols[k + l * 84].as_slice().to_vec())
                .collect::<Vec<f64>>()
        })
        .collect();
    na::DMatrix::from_column_slice(n_rows, 84, &data)
}
