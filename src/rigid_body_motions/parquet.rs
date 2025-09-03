use super::RigidBodyMotions;
use crate::Table;
use crate::{rigid_body_motions::RigidBodyMotionsError, LinearOpticalModelError};
use arrow::{
    array::{Float64Array, ListArray},
    datatypes::{DataType, Field, Float64Type, Schema},
    record_batch::RecordBatch,
};
use nalgebra as na;
use std::path::Path;
use std::sync::Arc;

type Result<T> = std::result::Result<T, LinearOpticalModelError>;

impl RigidBodyMotions {
    /// Creates a [RigidBodyMotions] from M1 and M2 rigid body motions saved in a [parquet](https://docs.rs/parquet) file
    pub fn from_parquet<P>(
        path: P,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<Self>
    where
        P: AsRef<Path>,
    {
        let table = Table::from_parquet(path)?;
        Self::from_table(&table, m1_rbm_label, m2_rbm_label)
    }
    /// Creates a [RigidBodyMotions] from a [Table]
    pub fn from_table(
        t: &Table,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<Self> {
        Self::from_record(&t.table(), m1_rbm_label, m2_rbm_label)
    }
    /// Creates a [RigidBodyMotions] from an Arrow table
    pub fn from_record(
        table: &RecordBatch,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<Self> {
        let schema = table.schema();
        // println!("{:#?}", schema.metadata());
        let idx = schema
            .index_of(m1_rbm_label.unwrap_or("OSSM1Lcl"))
            .map_err(|e| RigidBodyMotionsError::FromRecord(e.into()))?;
        let m1_rbm = table
            .column(idx)
            .as_any()
            .downcast_ref::<ListArray>()
            .unwrap();
        let idx = schema
            .index_of(m2_rbm_label.unwrap_or("MCM2Lcl6D"))
            .map_err(|e| RigidBodyMotionsError::FromRecord(e.into()))?;
        let m2_rbm = table
            .column(idx)
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
    /// Writes rigid body modtions to an Arrow table
    pub fn to_record(
        &self,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<RecordBatch> {
        let m1_rbm: Vec<_> = self
            .data
            .rows(0, 42)
            .column_iter()
            .map(|x| Some(x.as_slice().iter().map(|x| Some(*x)).collect::<Vec<_>>()))
            .collect();
        let m2_rbm: Vec<_> = self
            .data
            .rows(42, 42)
            .column_iter()
            .map(|x| Some(x.as_slice().iter().map(|x| Some(*x)).collect::<Vec<_>>()))
            .collect();
        let schema = Schema::new(vec![
            Field::new(
                m1_rbm_label.unwrap_or("M1RigidBodyMotions"),
                DataType::List(Arc::new(Field::new("item", DataType::Float64, true))),
                false,
            ),
            Field::new(
                m2_rbm_label.unwrap_or("M2RigidBodyMotions"),
                DataType::List(Arc::new(Field::new("item", DataType::Float64, true))),
                false,
            ),
        ]);
        Ok(RecordBatch::try_new(
            Arc::new(schema),
            vec![
                Arc::new(ListArray::from_iter_primitive::<Float64Type, _, _>(m1_rbm)),
                Arc::new(ListArray::from_iter_primitive::<Float64Type, _, _>(m2_rbm)),
            ],
        )
        .map_err(|e| RigidBodyMotionsError::ToRecord(e.into()))?)
    }
    /// Writes rigid body motions to a [Table]
    pub fn to_table(
        &self,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<Table> {
        self.to_record(m1_rbm_label, m2_rbm_label)
            .map(|record| record.into())
    }
    /// Writes rigid body motions to a Parquet file
    pub fn to_parquet(
        &self,
        path: impl AsRef<Path>,
        m1_rbm_label: Option<&str>,
        m2_rbm_label: Option<&str>,
    ) -> Result<()> {
        Ok(self
            .to_table(m1_rbm_label, m2_rbm_label)?
            .to_parquet(path)?)
    }
}
