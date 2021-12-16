pub struct RigidBodyMotions {
    pub time: Vec<f64>,
    pub data: nalgebra::DMatrix<f64>,
}

pub mod parquet {
    use super::RigidBodyMotions;
    use crate::Result;
    use arrow::{
        array::{Float64Array, ListArray},
        record_batch::RecordBatch,
    };
    use nalgebra as na;
    use parquet::arrow::{ArrowReader, ParquetFileArrowReader};
    use parquet::file::reader::SerializedFileReader;
    use std::{fs::File, path::Path, sync::Arc};

    impl RigidBodyMotions {
        pub fn from_parquet<P: AsRef<Path>>(path: P) -> Result<Self> {
            let file = File::open(path).unwrap();
            let file_reader = SerializedFileReader::new(file).unwrap();
            let mut arrow_reader = ParquetFileArrowReader::new(Arc::new(file_reader));
            let records = arrow_reader
                .get_record_reader(2048)
                .unwrap()
                .collect::<std::result::Result<Vec<RecordBatch>, arrow::error::ArrowError>>()
                .unwrap();
            let schema = records.get(0).unwrap().schema();
            let table = RecordBatch::concat(&schema, &records).unwrap();
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
                time,
                data: na::DMatrix::from_iterator(84, n, rbm.into_iter().flatten()),
            })
        }
    }
}
