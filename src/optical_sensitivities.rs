use std::{fmt::Display, ops::Deref};

use crate::{LinearOpticalModelError, Result};
#[cfg(feature = "faer")]
use faer_ext::IntoFaer;
use nalgebra as na;
use serde::{Deserialize, Serialize};

/// Optical sensitivities
///
/// Linear transformation of M1 and M2 rigid body motions into wavefront and wavefront piston and tip-tilt modes
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct OpticalSensitivities<const N: usize = 84>(Vec<OpticalSensitivity<N>>);
impl Deref for OpticalSensitivities {
    type Target = [OpticalSensitivity];
    fn deref(&self) -> &Self::Target {
        self.0.as_slice()
    }
}
impl<const N: usize> OpticalSensitivities<N> {
    /// Returns the wavefront within the exit pupil in `[m]`
    pub fn masked_wavefront(&self, data: &na::DMatrix<f64>) -> Vec<f64> {
        self[OpticalSensitivity::<N>::Wavefront(vec![])].into_optics(data)
    }
    /// Returns the wavefront in the exit pupil in `[rmm]`
    pub fn wavefront(&self, data: &na::DMatrix<f64>) -> Vec<f64> {
        let mut wavefront = self[OpticalSensitivity::<N>::Wavefront(vec![])]
            .into_optics(data)
            .into_iter();
        if let OpticalSensitivity::PupilMask(mask) =
            &self[OpticalSensitivity::<N>::PupilMask(vec![])]
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
impl<const N: usize> Display for OpticalSensitivities<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "Optical Sensitivities:")?;
        for s in &self.0 {
            writeln!(f, " * {s}")?;
        }
        Ok(())
    }
}
/// Optical sensitivity
///
/// Linear transformation of M1 and M2 rigid body motions into wavefront and wavefront piston and tip-tilt modes
#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum OpticalSensitivity<const N: usize = 84> {
    /// Wavefront sensitivity `[nxN]` where n in the pupil resolution
    Wavefront(Vec<f64>),
    /// Exit pupil tip-tilt sensitivity `[2xN]`
    TipTilt(Vec<f64>),
    /// Exit pupil segment tip-tilt `[14xN]`
    SegmentTipTilt(Vec<f64>),
    /// Exit pupil segment piston `[7xN]`
    SegmentPiston(Vec<f64>),
    SegmentMask(Vec<i32>),
    PupilMask(Vec<bool>),
}
impl<const N: usize> Display for OpticalSensitivity<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            OpticalSensitivity::Wavefront(_) => write!(f, "Wavefront"),
            OpticalSensitivity::TipTilt(_) => write!(f, "TipTilt"),
            OpticalSensitivity::SegmentTipTilt(_) => write!(f, "SegmentTipTilt"),
            OpticalSensitivity::SegmentPiston(_) => write!(f, "SegmentPiston"),
            OpticalSensitivity::SegmentMask(_) => write!(f, "SegmentMask"),
            OpticalSensitivity::PupilMask(_) => write!(f, "PupilMask"),
        }
    }
}
impl<'a, const N: usize> From<&'a OpticalSensitivity<N>> for na::DMatrix<f64> {
    fn from(sens: &'a OpticalSensitivity<N>) -> Self {
        use OpticalSensitivity::*;
        match sens {
            Wavefront(val) => Some(na::DMatrix::from_column_slice(val.len() / N, N, val)),
            TipTilt(val) => Some(na::DMatrix::from_column_slice(2, N, val)),
            SegmentTipTilt(val) => Some(na::DMatrix::from_column_slice(14, N, val)),
            SegmentPiston(val) => Some(na::DMatrix::from_column_slice(7, N, val)),
            _ => None,
        }
        .unwrap()
    }
}
impl<const N: usize> PartialEq<OpticalSensitivity<N>> for OpticalSensitivity<N> {
    fn eq(&self, other: &OpticalSensitivity<N>) -> bool {
        use OpticalSensitivity::*;
        match (self, other) {
            (Wavefront(_), Wavefront(_)) => true,
            (TipTilt(_), TipTilt(_)) => true,
            (SegmentTipTilt(_), SegmentTipTilt(_)) => true,
            (SegmentPiston(_), SegmentPiston(_)) => true,
            (SegmentMask(_), SegmentMask(_)) => true,
            (PupilMask(_), PupilMask(_)) => true,
            _ => false,
        }
    }
}
impl<const N: usize> std::ops::Index<OpticalSensitivity<N>> for OpticalSensitivities<N> {
    type Output = OpticalSensitivity<N>;

    fn index(&self, index: OpticalSensitivity<N>) -> &Self::Output {
        self.0
            .iter()
            // .find_map(|s| if index == *s { Some(s) } else { None })
            .find(|&s| index == *s)
            .expect(&format!("cannot find optical sensitivity: {}", index))
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

impl<const N: usize> OpticalSensitivity<N> {
    /// Returns M1 wavefront sensitivities `[nx42]`
    pub fn m1_wavefront(&self) -> Result<na::DMatrix<f64>> {
        match self {
            OpticalSensitivity::Wavefront(sens) => {
                let n = sens.len() / N;
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
    #[cfg(not(feature = "faer"))]
    pub fn into_optics(&self, rbm: &na::DMatrix<f64>) -> Vec<f64> {
        match self {
            /*OpticalSensitivity::Wavefront(sens) => {
                let n = sens.len() / N;
                //println!("n: {}", n);
                let sensitivity = na::DMatrix::from_column_slice(n, N, sens);
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
                let sensitivity = na::DMatrix::from_column_slice(2, N, sens);
                let tip_tilt = sensitivity * rbm;
                tip_tilt.as_slice().to_owned()
            }
            OpticalSensitivity::SegmentTipTilt(sens) => {
                let sensitivity = na::DMatrix::from_column_slice(14, N, sens);
                let segment_tip_tilt = sensitivity * rbm;
                segment_tip_tilt.as_slice().to_owned()
            }
            OpticalSensitivity::SegmentPiston(sens) => {
                let sensitivity = na::DMatrix::from_column_slice(7, N, sens);
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
            OpticalSensitivity::Wavefront(sens) => {
                let sensitivity = na::DMatrix::from_column_slice(sens.len() / N, N, sens);
                let wavefront = sensitivity * rbm;
                wavefront.as_slice().to_owned()
            }
            _ => unimplemented!(),
        }
    }

    #[cfg(feature = "faer")]
    pub fn into_optics(&self, rbm: &na::DMatrix<f64>) -> Vec<f64> {
        let mat = match self {
            /*OpticalSensitivity::Wavefront(sens) => {
                let n = sens.len() / N;
                //println!("n: {}", n);
                let sensitivity = na::DMatrix::from_column_slice(n, N, sens);
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
                let sensitivity = faer::MatRef::from_column_major_slice(sens, 2, N);
                sensitivity * rbm.view_range(.., ..).into_faer()
            }
            OpticalSensitivity::SegmentTipTilt(sens) => {
                let sensitivity = faer::MatRef::from_column_major_slice(sens, 14, N);
                sensitivity * rbm.view_range(.., ..).into_faer()
            }
            OpticalSensitivity::SegmentPiston(sens) => {
                let sensitivity = faer::MatRef::from_column_major_slice(sens, 7, N);
                sensitivity * rbm.view_range(.., ..).into_faer()
                // let mut v: Vec<f64> = vec![];
                // for (k, row) in segment_piston.row_iter().take(6).enumerate() {
                //     //println!("{}: {:?}", k, row.shape());
                //     v.extend(
                //         &mut segment_piston
                //             .rows(k + 1, 6 - k)
                //             .row_iter()
                //             .flat_map(|y| (y - row).as_slice().to_owned()),
                //     );
                // }
            }
            OpticalSensitivity::Wavefront(sens) => {
                let sensitivity = faer::MatRef::from_column_major_slice(sens, sens.len() / N, N);
                sensitivity * rbm.view_range(.., ..).into_faer()
            }
            _ => unimplemented!(),
        };
        let n = mat.nrows() * mat.ncols();
        let mut dst = Vec::with_capacity(n);
        unsafe {
            std::ptr::copy(mat.as_ptr(), dst.as_mut_ptr(), n);
            dst.set_len(n);
        }
        dst
    }

    /*
       pub fn transform(&self, optics_model: &WindLoadedGmtInner) -> OpticalWindLoad {
           let n_sample = optics_model.n_sample;
           let rbm = &optics_model.rbm;
           match self {
               OpticalSensitivity::Wavefront(sens) => {
                   let n = sens.len() / N;
                   //println!("n: {}", n);
                   let sensitivity = na::DMatrix::from_column_slice(n, N, sens);
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
                   let sensitivity = na::DMatrix::from_column_slice(2, N, sens);
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
                   let sensitivity = na::DMatrix::from_column_slice(14, N, sens);
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
                   let sensitivity = na::DMatrix::from_column_slice(7, N, sens);
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
pub fn from_opticals<const N: usize>(senses: &[OpticalSensitivity<N>]) -> na::DMatrix<f64> {
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
    let data: Vec<_> = (0..N)
        .flat_map(|k| {
            (0..senses.len())
                .flat_map(|l| cols[k + l * N].as_slice().to_vec())
                .collect::<Vec<f64>>()
        })
        .collect();
    na::DMatrix::from_column_slice(n_rows, N, &data)
}
