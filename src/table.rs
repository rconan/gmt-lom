use crate::Result;
use arrow::compute::concat_batches;
use arrow::record_batch::{RecordBatch, RecordBatchReader};
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use std::{fs::File, path::Path};

pub struct Table {
    record: RecordBatch,
}

impl Table {
    pub fn from_parquet<P>(path: P) -> Result<Self>
    where
        P: AsRef<Path>,
    {
        let file = File::open(path)?;
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
