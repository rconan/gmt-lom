//! # Derivation of optical sensitivities
//!
//! Computes the optical sensitivity matrices as a function of M1 and M2
//! rigid body motions and saves the matrices in the file `optical_sensitivities.rs.bin`
//! It requires the `crseo` feature and a NVIDIA GPU and is run with:
//!
//! `cargo run --release --bin makesens --features crseo`

use gmt_lom::{Bin, OpticalSensitivities};
fn main() -> anyhow::Result<()> {
    OpticalSensitivities::compute(None)?.dump("optical_sensitivities.rs.bin")?;
    println!("Sensitivities written to optical_sensitivities.rs.bin");
    Ok(())
}
