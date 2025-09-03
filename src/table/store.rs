use std::sync::Arc;

use arrow::{compute::concat_batches, error::ArrowError};
use futures::TryStreamExt;
use object_store::ObjectStore;
use parquet::{
    arrow::{async_reader::ParquetObjectReader, ParquetRecordBatchStreamBuilder},
    errors::ParquetError,
};

use crate::{LinearOpticalModelError, Table};

#[derive(Debug, thiserror::Error)]
pub enum StoredTableError {
    #[error("failed to read parquet S3 object: {1}")]
    ReadParquet(#[source] ParquetError, String),
    #[error("failed to stream parquet data from S3")]
    StreamParquet(#[from] ParquetError),
    #[error("failed to collect Arrow record batches")]
    Records(#[from] ArrowError),
    #[error("empty Arrow records")]
    Empty,
}

impl From<StoredTableError> for LinearOpticalModelError {
    fn from(s3t: StoredTableError) -> Self {
        Self::TableRead(s3t.into())
    }
}

impl Table {
    pub async fn from_stored_parquet(
        store: impl ObjectStore,
        object_path: impl Into<object_store::path::Path>,
    ) -> Result<Self, LinearOpticalModelError> {
        let object_path = object_path.into();
        let reader = ParquetObjectReader::new(Arc::new(store), object_path.clone());
        let stream = ParquetRecordBatchStreamBuilder::new(reader)
            .await
            .map_err(|e| StoredTableError::ReadParquet(e, object_path.to_string()))?
            .build()
            .map_err(|e| StoredTableError::from(e))?;
        let results = stream
            .try_collect::<Vec<_>>()
            .await
            .map_err(|e| StoredTableError::from(e))?;

        if results.is_empty() {
            return Err(StoredTableError::Empty.into());
        }

        let record = concat_batches(results.get(0).unwrap().schema_ref(), results.as_slice())
            .map_err(|e| StoredTableError::from(e))?;
        Ok(Self { record })
    }
}
