use crate::Formatting;
use skyangle::Conversion;
use std::{fmt::Display, iter::FromIterator};

/// GMT M1 and M2 segment rigid body motions
///
/// The rigid body motions are saved in a matrix with 84 rows and as many columns as the number of time steps
/// A row has the following format: `[M2,M1]` where `[Mi]=[S1,S2,S3,S4,S5,S6,S7]` and `[Sj]=[Tjx,Tjy,Tjz,Rjx,Rjy,Rjz]`
pub struct RigidBodyMotions {
    // sampling frequency
    sampling_frequency: Option<f64>,
    // time vector
    time: Option<Vec<f64>>,
    // `[84,n]` matrix of rigid body motion
    data: nalgebra::DMatrix<f64>,
    pub format: Formatting,
}
impl Display for RigidBodyMotions {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        //let mean = self.data.row_mean();
        let var = self.data.column_variance();
        for (i, var) in var
            .into_iter()
            .cloned()
            .collect::<Vec<f64>>()
            .chunks(42)
            .enumerate()
        {
            writeln!(f, "M{:} RBM:", i + 1)?;
            for (j, var) in var.chunks(6).enumerate() {
                match self.format {
                    Formatting::AdHoc => {
                        let t_xyz: Vec<_> = var[..3].iter().map(|x| x.sqrt() * 1e9).collect();
                        let r_xyz: Vec<_> = var[3..].iter().map(|x| x.sqrt().to_mas()).collect();
                        writeln!(f, " - #{:} {:6.0?} {:6.0?}", j + 1, t_xyz, r_xyz)?
                    }
                    Formatting::Latex => {
                        let t_xyz: Vec<_> = var[..3]
                            .iter()
                            .map(|x| x.sqrt() * 1e9)
                            .map(|x| format!("{:6.0}", x))
                            .collect();
                        let r_xyz: Vec<_> = var[3..]
                            .iter()
                            .map(|x| x.sqrt().to_mas())
                            .map(|x| format!("{:6.0}", x))
                            .collect();
                        writeln!(
                            f,
                            r" {:} &  {:} & {:} \\",
                            j + 1,
                            t_xyz.join(" & "),
                            r_xyz.join(" & ")
                        )?
                    }
                };
            }
        }
        Ok(())
    }
}
/// Creates a [RigidBodyMotions] from an iterator of [tuple] of M1 and M2 7 segments [Vec] of 6 rigid body motions (Txyz and Rxyz)
impl FromIterator<(Vec<Vec<f64>>, Vec<Vec<f64>>)> for RigidBodyMotions {
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = (Vec<Vec<f64>>, Vec<Vec<f64>>)>,
    {
        let data: Vec<f64> = iter
            .into_iter()
            .map(|(m1, m2)| {
                (
                    m1.into_iter().flatten().collect::<Vec<f64>>(),
                    m2.into_iter().flatten().collect::<Vec<f64>>(),
                )
            })
            .flat_map(|(m1, m2)| m1.into_iter().chain(m2.into_iter()).collect::<Vec<f64>>())
            .collect();
        Self {
            sampling_frequency: None,
            time: None,
            data: nalgebra::DMatrix::from_vec(84, data.len() / 84, data),
            format: Formatting::AdHoc,
        }
    }
}
/*
/// Creates a [RigidBodyMotions] from an iterator of [tuple] of M1 and M2 [Vec] of 42 rigid body motions
impl FromIterator<(Vec<f64>, Vec<f64>)> for RigidBodyMotions {
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = (Vec<f64>, Vec<f64>)>,
    {
        let data: Vec<f64> = iter
            .into_iter()
            .flat_map(|(m1, m2)| m1.into_iter().chain(m2.into_iter()).collect::<Vec<f64>>())
            .collect();
        Self {
            sampling_frequency: None,
            time: None,
            data: nalgebra::DMatrix::from_vec(84, data.len() / 84, data),
        }
    }
}
*/
/// Creates a [RigidBodyMotions] from an iterator of [tuple] of M1 and M2 [slice] of 42 rigid body motions
impl<'a> FromIterator<(&'a [f64], &'a [f64])> for RigidBodyMotions {
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = (&'a [f64], &'a [f64])>,
    {
        let data: Vec<f64> = iter
            .into_iter()
            .flat_map(|(m1, m2)| {
                m1.iter()
                    .cloned()
                    .chain(m2.iter().cloned())
                    .collect::<Vec<f64>>()
            })
            .collect();
        Self {
            sampling_frequency: None,
            time: None,
            data: nalgebra::DMatrix::from_vec(84, data.len() / 84, data),
            format: Formatting::AdHoc,
        }
    }
}
impl RigidBodyMotions {
    /// Returns the time vector
    pub fn time(&self) -> Vec<f64> {
        if let Some(time) = &self.time {
            time.to_vec()
        } else {
            let tau = self.sampling_frequency.unwrap_or(1f64).recip();
            (0..self.data.ncols()).map(|i| tau * i as f64).collect()
        }
    }
    /// Returns the number of rigidbody motions sample `n`
    pub fn len(&self) -> usize {
        self.data.ncols()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Returns a reference to the rigid body motion `[84,n]` matrix
    pub fn data(&self) -> &nalgebra::DMatrix<f64> {
        &self.data
    }
    /// Consumes the object and returns the rigid body motion `[84,n]` matrix
    pub fn into_data(self) -> nalgebra::DMatrix<f64> {
        self.data
    }
}

#[cfg(feature = "apache")]
pub mod parquet {
    use super::RigidBodyMotions;
    use crate::{Result, Table};
    use arrow::array::{Float64Array, ListArray};
    use nalgebra as na;
    use std::path::Path;

    impl RigidBodyMotions {
        /// Creates a [RigidBodyMotions] from M1 and M2 rigid body motions saved in a [parquet](https://docs.rs/parquet) file
        pub fn from_parquet<P>(path: P) -> Result<Self>
        where
            P: AsRef<Path>,
        {
            let table = Table::from_parquet(path)?;
            Self::from_table(&table)
        }
        /// Creates a [RigidBodyMotions] from M1 and M2 rigid body motions saved in a [parquet](https://docs.rs/parquet) table
        pub fn from_table(t: &Table) -> Result<Self> {
            let table = t.table();
            let m1_rbm = table
                .column(0)
                .as_any()
                .downcast_ref::<ListArray>()
                .unwrap();
            let m2_rbm = table
                .column(1)
                .as_any()
                .downcast_ref::<ListArray>()
                .unwrap();
            let (time, rbm): (Vec<f64>, Vec<Vec<f64>>) = m1_rbm
                .iter()
                .zip(m2_rbm.iter())
                .enumerate()
                .filter_map(|(k, (m1, m2))| match (m1, m2) {
                    (Some(m1_rbm), Some(m2_rbm)) => {
                        let m1_rbm_data = m1_rbm
                            .as_any()
                            .downcast_ref::<Float64Array>()
                            .unwrap()
                            .iter()
                            .collect::<Option<Vec<f64>>>();
                        let m2_rbm_data = m2_rbm
                            .as_any()
                            .downcast_ref::<Float64Array>()
                            .unwrap()
                            .iter()
                            .collect::<Option<Vec<f64>>>();
                        if let (Some(m1_rbm_data), Some(m2_rbm_data)) = (m1_rbm_data, m2_rbm_data) {
                            Some((
                                k as f64,
                                m1_rbm_data
                                    .into_iter()
                                    .chain(m2_rbm_data.into_iter())
                                    .collect::<Vec<f64>>(),
                            ))
                        } else {
                            None
                        }
                    }
                    _ => None,
                })
                .unzip();
            let n = time.len();
            Ok(Self {
                sampling_frequency: Some((time[1] - time[0]).recip()),
                time: Some(time),
                data: na::DMatrix::from_iterator(84, n, rbm.into_iter().flatten()),
                format: super::Formatting::AdHoc,
            })
        }
    }
}
