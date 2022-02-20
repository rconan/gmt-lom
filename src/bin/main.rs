//! # Tip-tilt derivation from M1 & M2 rigid body motions in a parquet file
//!
//! The application takes 2 optional argument:
//!  1. the path to the parquet file <".">
//!  2. the parquet file name without the ".parquet" extension <"data">

use gmt_lom::{Loader, OpticalMetrics, Stats, LOM};
use std::env::args;

fn main() -> anyhow::Result<()> {
    let mut data_path = vec![".".to_string(), "data".to_string()];
    for (k, arg) in args().skip(1).enumerate() {
        data_path[k] = arg;
    }
    let filename = format!("{}.parquet", data_path[1]);
    let lom = LOM::builder()
        .load_rigid_body_motions(Loader::default().path(&data_path[0]).filename(&filename))?
        .build()?;
    let tiptilt = lom.tiptilt();

    let n_sample = 1000 * 90;

    println!("TT STD.: {:.0?}mas", tiptilt.std(Some(n_sample)));
    println!(
        "Segment TT STD.: {:.0?}mas",
        lom.segment_tiptilt().std(Some(60_000))
    );
    let segment_piston = lom.segment_piston();
    println!(
        "Segment Piston STD.: {:.0?}nm",
        segment_piston.std(Some(n_sample))
    );

    let _: complot::Plot = (
        lom.time()
            .iter()
            .zip(tiptilt.items())
            .map(|(&t, xy)| (t * 1e-3, xy.to_vec())),
        complot::complot!(
            "lom_tiptilt.png",
            xlabel = "Time [s]",
            ylabel = "Tip-Tilt [arcsec]"
        ),
    )
        .into();

    let _: complot::Plot = (
        lom.time()
            .iter()
            .zip(segment_piston.items())
            .map(|(&t, xy)| (t * 1e-3, xy.to_vec())),
        complot::complot!(
            "lom_segment-piston.png",
            xlabel = "Time [s]",
            ylabel = "Piston [nm]"
        ),
    )
        .into();
    Ok(())
}
