use arrow::compute::concat_batches;
use arrow::record_batch::{RecordBatch, RecordBatchReader};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use std::path::PathBuf;
use std::{fs::File, path::Path};

#[derive(Debug, thiserror::Error)]
pub enum TableError {
    #[cfg(feature = "object_store")]
    #[error("failed to load a parquet table from an S3 bucket")]
    S3Table(#[from] store::StoredTableError),
    #[error("Parqutet file: {1:?} not found")]
    ParquetFile(#[source] std::io::Error, PathBuf),
    #[error("parquet read failed")]
    Parquet(#[from] parquet::errors::ParquetError),
    #[error("arrow record get failed")]
    Arrow(#[from] arrow::error::ArrowError),
}

pub struct Table {
    record: RecordBatch,
}

impl Table {
    pub fn from_parquet<P>(path: P) -> Result<Self, TableError>
    where
        P: AsRef<Path>,
    {
        let file = File::open(&path)
            .map_err(|e| TableError::ParquetFile(e, path.as_ref().to_path_buf()))?;
        let parquet_reader = ParquetRecordBatchReaderBuilder::try_new(file)?
            .with_batch_size(2048)
            .build()?;
        let schema = parquet_reader.schema();
        let records: std::result::Result<Vec<_>, arrow::error::ArrowError> =
            parquet_reader.collect();
        let record = concat_batches(&schema, records?.as_slice())?;
        Ok(Self { record })
    }
    pub fn table(&self) -> &RecordBatch {
        &self.record
    }
}

impl From<RecordBatch> for Table {
    fn from(record: RecordBatch) -> Self {
        Self { record }
    }
}

#[cfg(feature = "object_store")]
pub mod store;
