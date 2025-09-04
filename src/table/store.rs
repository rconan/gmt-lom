use std::sync::Arc;

use arrow::{compute::concat_batches, error::ArrowError};
use bytes::Bytes;
use futures::TryStreamExt;
use object_store::{multipart::MultipartStore, ObjectStore};
use parquet::{
    arrow::{async_reader::ParquetObjectReader, ParquetRecordBatchStreamBuilder},
    errors::ParquetError,
};

use crate::{LinearOpticalModelError, Table};

#[derive(Debug, thiserror::Error)]
pub enum StoredTableError {
    #[error("failed to upload S3 object: {1}")]
    StorePut(#[source] object_store::Error, String),
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
    /// Loads a table from a parquet stored remotely in [store](https://docs.rs/object_store/latest/object_store/trait.ObjectStore.html)
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
    /// Saves a table to a parquet stored remotely in [store](https://docs.rs/object_store/latest/object_store/trait.ObjectStore.html)
    pub async fn to_stored_parquet(
        &self,
        store: impl ObjectStore + MultipartStore,
        object_path: impl Into<object_store::path::Path>,
    ) -> Result<(), LinearOpticalModelError> {
        let object_path = object_path.into();
        let bytes: Bytes = self.to_mem()?.into();
        const CHUNK_SIZE: usize = 5 * 1024 * 1024; // 5MB minimum
        if bytes.len() > CHUNK_SIZE {
            let upload = store
                .create_multipart(&object_path)
                .await
                .map_err(|e| StoredTableError::StorePut(e, object_path.to_string()))?;

            let mut parts = Vec::new();
            let mut part_number = 0;

            for chunk in bytes.chunks(CHUNK_SIZE) {
                let chunk_bytes = Bytes::copy_from_slice(chunk);
                let part = store
                    .put_part(&object_path, &upload, part_number, chunk_bytes.into())
                    .await
                    .map_err(|e| StoredTableError::StorePut(e, object_path.to_string()))?;
                parts.push(part);
                part_number += 1;
            }

            store
                .complete_multipart(&object_path, &upload, parts)
                .await
                .map_err(|e| StoredTableError::StorePut(e, object_path.to_string()))?;
        } else {
            store
                .put(&object_path, bytes.into())
                .await
                .map_err(|e| StoredTableError::StorePut(e, object_path.to_string()))?;
        }
        Ok(())
    }
}
