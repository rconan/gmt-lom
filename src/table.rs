use crate::Result;
use arrow::record_batch::RecordBatch;
use parquet::arrow::{ArrowReader, ParquetFileArrowReader};
use parquet::file::reader::SerializedFileReader;
use std::{fs::File, path::Path, sync::Arc};

pub struct Table {
    table: RecordBatch,
}

impl Table {
    pub fn from_parquet<P>(path: P) -> Result<Self>
    where
        P: AsRef<Path>,
    {
        let file = File::open(path)?;
        let file_reader = SerializedFileReader::new(file)?;
        let mut arrow_reader = ParquetFileArrowReader::new(Arc::new(file_reader));
        let records = arrow_reader
            .get_record_reader(2048)
            .unwrap()
            .collect::<std::result::Result<Vec<RecordBatch>, arrow::error::ArrowError>>()?;
        let schema = records.get(0).unwrap().schema();
        let table = RecordBatch::concat(&schema, &records)?;
        Ok(Self { table })
    }
    pub fn table(&self) -> &RecordBatch {
        &self.table
    }
}
