use crate::{LinearOpticalModelError, Result, SegmentPiston, SegmentTipTilt, Table};
use arrow::array::{Float64Array, ListArray};
use csv;
use nalgebra as na;
use serde::Deserialize;
use std::path::Path;

pub struct Pmt {
    pub sampling_frequency: Option<f64>,
    pub time: Option<Vec<f64>>,
    data: na::DMatrix<f64>,
}
impl Pmt {
    /// Creates a [RigidBodyMotions] from M1 and M2 rigid body motions saved in a [parquet](https://docs.rs/parquet) table
    pub fn from_table(t: &Table) -> Result<Self> {
        let table = t.table();
        let (col, _) = table
            .schema()
            .column_with_name("PMT3D")
            .ok_or(LinearOpticalModelError::Table("PMT3D".to_string()))?;
        let pmt = table
            .column(col)
            .as_any()
            .downcast_ref::<ListArray>()
            .unwrap();
        let (time, rbm): (Vec<f64>, Vec<Vec<f64>>) = pmt
            .iter()
            .enumerate()
            .filter_map(|(k, pmt)| match pmt {
                Some(pmt) => {
                    let pmt_data = pmt
                        .as_any()
                        .downcast_ref::<Float64Array>()
                        .unwrap()
                        .iter()
                        .collect::<Option<Vec<f64>>>();
                    if let Some(pmt_data) = pmt_data {
                        Some((k as f64, pmt_data))
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
            data: na::DMatrix::from_iterator(300, n, rbm.into_iter().flatten()),
        })
    }
    pub fn shuffle(&mut self, idx: Vec<usize>) {
        let data: Vec<_> = idx.into_iter().map(|i| self.data.row(i)).collect();
        self.data = na::DMatrix::from_rows(&data);
    }
    pub fn segment_tiptilt(&self) -> Result<SegmentTipTilt> {
        let pmt_sens: na::DMatrix<f64> =
            PmtSensitivity::new("pmts/GMT-DTA-190951_RevB_pmt1.csv")?.into();
        let segment_tiptilt = pmt_sens * &self.data;
        Ok(SegmentTipTilt(
            segment_tiptilt.map(|x| x * 1e3).as_slice().to_vec(),
        ))
    }
    pub fn segment_piston(&self) -> Result<SegmentPiston> {
        let pmt_sens: na::DMatrix<f64> =
            PmtSensitivity::new("pmts/GMT-DTA-190951_RevB_pmt2.csv")?.into();
        let segment_piston = pmt_sens * &self.data;
        Ok(SegmentPiston(
            segment_piston.map(|x| x * 1e9).as_slice().to_vec(),
        ))
    }
}
#[derive(Debug)]
pub struct PmtSensitivity {
    pub data: Vec<f64>,
    pub n_row: usize,
    pub n_col: usize,
}
impl From<PmtSensitivity> for na::DMatrix<f64> {
    fn from(pmt_sens: PmtSensitivity) -> Self {
        na::DMatrix::from_row_slice(pmt_sens.n_row, pmt_sens.n_col, &pmt_sens.data)
    }
}

#[derive(Deserialize)]
struct Row {
    #[allow(dead_code)]
    metric: String,
    #[allow(dead_code)]
    segment: String,
    values: Vec<f64>,
}

impl PmtSensitivity {
    pub fn new<P: AsRef<Path>>(path: P) -> Result<Self> {
        let mut rdr = csv::ReaderBuilder::new()
            .has_headers(false)
            .from_path(path)?;
        let values: Result<Vec<_>> = rdr
            .deserialize()
            .map(|result| {
                let record: Row = result?;
                Ok(record.values)
            })
            .collect();
        values.map(|values| {
            let n_row = values.len();
            let n_col = values[0].len();
            Self {
                data: values.into_iter().flatten().collect(),
                n_row,
                n_col,
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pmts1() {
        match PmtSensitivity::new("pmts/GMT-DTA-190951_RevB_pmt1.csv") {
            Ok(pmt) => {
                assert_eq!(pmt.n_row, 14);
                assert_eq!(pmt.n_col, 300);
            }
            Err(e) => panic!("{e}"),
        }
    }
    #[test]
    fn pmts2() {
        match PmtSensitivity::new("pmts/GMT-DTA-190951_RevB_pmt2.csv") {
            Ok(pmt) => {
                assert_eq!(pmt.n_row, 7);
                assert_eq!(pmt.n_col, 300);
            }
            Err(e) => panic!("{e}"),
        }
    }
}
