use gmt_lom::{Bin, OpticalSensitivities};
fn main() -> anyhow::Result<()> {
    OpticalSensitivities::compute(None)?.dump("optical_sensitivities.rs.bin")?;
    println!("Sensitivities written to optical_sensitivities.rs.bin");
    Ok(())
}
