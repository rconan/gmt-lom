//! # Tip-tilt derivation from M1 & M2 rigid body motions in a parquet file
//!
//! The application takes 2 optional argument:
//!  1. the path to the parquet file <".">
//!  2. the parquet file name without the ".parquet" extension <"data">

use gmt_lom::{OpticalMetrics, Stats, Table, ToPkl, LOM};
use skyangle::Conversion;
use std::path::Path;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(
    name = "GMT Linear Optical Model",
    about = "GMT M1/M2 rigid body motions to optics linear transformations"
)]
struct Opt {
    /// Path to the rigid body motions parquet file
    #[structopt(short, long, default_value = ".")]
    path: String,
    /// Name of the parquet data file
    #[structopt(short, long, default_value = "data.parquet")]
    file: String,
    /// Data sampling frequency [Hz]
    #[structopt(short, long)]
    sampling_frequency: Option<f64>,
    /// Compute statistics on the last n seconds
    #[structopt(short, long)]
    last: Option<f64>,
    /// Compute the tip and tilt PSDs (requires the "welch-sde" feature)
    #[structopt(long)]
    tip_tilt_psds: bool,
    /// Compute the segment piston PSDs (requires the "welch-sde" feature)
    #[structopt(long)]
    segment_piston_psds: bool,
    /// Save the tip and tilte to a pickle file
    #[structopt(long)]
    tiptilt_pickle: Option<String>,
    /// Save the segment piston to a pickle file
    #[structopt(long)]
    segment_piston_pickle: Option<String>,
    /// Format output for insertion into Latex tables
    #[structopt(long)]
    latex: bool,
    #[structopt(long)]
    zm1: bool,
    /// Set M2 RBM to zero
    #[structopt(long)]
    zm2: bool,
}

fn main() -> anyhow::Result<()> {
    let opt = Opt::from_args();

    let path = Path::new(&opt.path);
    let table = Table::from_parquet(path.join(opt.file))?;

    let mut lom = LOM::builder()
        .table_rigid_body_motions(
            &table,
            Some("M1RigidBodyMotions"),
            Some("M2RigidBodyMotions"),
        )?
        .build()?;
    if opt.zm1 {
        lom.rbm.zeroed_m1()
    }
    if opt.zm2 {
        lom.rbm.zeroed_m2()
    }
    if opt.latex {
        lom.latex();
    }
    println!("{lom}");
    let tiptilt = lom.tiptilt();
    if let Some(tiptilt_file) = opt.tiptilt_pickle {
        tiptilt.to_pkl(tiptilt_file)?;
    }

    let n_sample = match (opt.last, opt.sampling_frequency) {
        (None, None) => Result::<usize, anyhow::Error>::Ok(lom.len()),
        (_, None) | (None, _) => {
            anyhow::bail!("Either last or sampling-frequency option is missing")
        }
        (Some(l), Some(s)) => Ok((s * l).round() as usize),
    }?;

    println!("TT STD.: {:.0?}mas", tiptilt.std(Some(n_sample)).to_mas());
    println!(
        "Segment TT STD.: {:.0?}mas",
        lom.segment_tiptilt().std(Some(n_sample)).to_mas()
    );
    let segment_piston = lom.segment_piston();
    if let Some(segment_piston_file) = opt.segment_piston_pickle {
        segment_piston.to_pkl(segment_piston_file)?;
    }
    println!(
        "Segment Piston STD.: {:.0?}nm",
        segment_piston
            .std(Some(n_sample))
            .into_iter()
            .map(|x| x * 1e9)
            .collect::<Vec<f64>>()
    );

    let n = lom.len() - n_sample;
    let _: complot::Plot = (
        lom.time()
            .iter()
            .zip(tiptilt.items())
            .map(|(&t, xy)| (t * 1e-3, xy.to_vec())),
        complot::complot!(
            "lom_tiptilt.png",
            xlabel = "Time [s]",
            ylabel = "Tip-Tilt [mas]"
        ),
    )
        .into();

    let _: complot::Plot = (
        lom.time()
            .iter()
            .zip(segment_piston.items())
            .skip(n)
            .map(|(&t, xy)| (t * 1e-3, xy.to_vec())),
        complot::complot!(
            "lom_segment-piston.png",
            xlabel = "Time [s]",
            ylabel = "Piston [nm]"
        ),
    )
        .into();

    if opt.tip_tilt_psds {
        let n = lom.len() - n_sample;
        let (tip, tilt): (Vec<f64>, Vec<f64>) = {
            let (tip, tilt): (Vec<_>, Vec<_>) =
                tiptilt.items().skip(n).map(|xy| (xy[0], xy[1])).unzip();
            let mean_tip = tip.iter().cloned().sum::<f64>() / tip.len() as f64;
            let mean_tilt = tilt.iter().cloned().sum::<f64>() / tilt.len() as f64;
            (
                tip.into_iter().map(|x| 1e-3 * (x - mean_tip)).collect(),
                tilt.into_iter().map(|x| 1e-3 * (x - mean_tilt)).collect(),
            )
        };
        use welch_sde::{Build, PowerSpectrum};
        let welch: PowerSpectrum<f64> = PowerSpectrum::builder(&tip)
            .sampling_frequency(opt.sampling_frequency.unwrap_or(1f64))
            .dft_log2_max_size(10)
            .build();
        println!("{welch}");
        let tip_psd = welch.periodogram();
        let welch: PowerSpectrum<f64> = PowerSpectrum::builder(&tilt)
            .sampling_frequency(opt.sampling_frequency.unwrap_or(1f64))
            .dft_log2_max_size(10)
            .build();
        let tilt_psd = welch.periodogram();
        /*
                let n = lom.len() - n_sample;
                let (mut tiptilt, mut tilt): (Vec<_>, Vec<_>) =
                    tiptilt.items().skip(n).map(|xy| (xy[0], xy[1])).unzip();
                tiptilt.append(&mut tilt);
                use welch_sde::{Build, PowerSpectrum};
                let welch: PowerSpectrum<f64> = PowerSpectrum::builder(&tiptilt)
                    .n_signal(2)
                    .sampling_frequency(opt.sampling_frequency.unwrap_or(1f64))
                    .build();
                println!("{welch}");
                let tiptilt_psd = welch.periodogram();
                let h = tiptilt_psd.len() / 2;
                let (tip_psd, tilt_psd) = tiptilt_psd.split_at(h);
        */

        let _: complot::LogLog = (
            tip_psd
                .frequency()
                .into_iter()
                .zip(tip_psd.iter().zip(tilt_psd.iter()))
                .skip(1)
                .map(|(f, (x, y))| (f, vec![*x, *y])),
            complot::complot!(
                "lom_tiptilt-psds.png",
                xlabel = "Frequency [Hz]",
                ylabel = "PSD [mas^2/Hz]"
            ),
        )
            .into();
    }
    if opt.segment_piston_psds {
        /*
               let n = lom.len() - n_sample;
               let (tip, tilt): (Vec<f64>, Vec<f64>) = {
                   let (tip, tilt): (Vec<_>, Vec<_>) =
                       tiptilt.items().skip(n).map(|xy| (xy[0], xy[1])).unzip();
                   let mean_tip = tip.iter().cloned().sum::<f64>() / tip.len() as f64;
                   let mean_tilt = tilt.iter().cloned().sum::<f64>() / tilt.len() as f64;
                   (
                       tip.into_iter().map(|x| 1e-3 * (x - mean_tip)).collect(),
                       tilt.into_iter().map(|x| 1e-3 * (x - mean_tilt)).collect(),
                   )
               };
               use welch_sde::{Build, PowerSpectrum};
               let welch: PowerSpectrum<f64> = PowerSpectrum::builder(&tip)
                   .sampling_frequency(opt.sampling_frequency.unwrap_or(1f64))
                   .build();
               println!("{welch}");
               let tip_psd = welch.periodogram();
               let welch: PowerSpectrum<f64> = PowerSpectrum::builder(&tilt)
                   .sampling_frequency(opt.sampling_frequency.unwrap_or(1f64))
                   .build();
               let tilt_psd = welch.periodogram();
        let n = lom.len() - n_sample;
        let segments_piston = segment_piston.time_wise(Some(n_sample));
        use welch_sde::{Build, PowerSpectrum};
        let welch: PowerSpectrum<f64> = PowerSpectrum::builder(&segments_piston)
            .n_signal(7)
            .sampling_frequency(opt.sampling_frequency.unwrap_or(1f64))
            .build();
        println!("{welch}");
        let segments_piston_psd = welch.periodogram();
        let h = segments_piston_psd.len() / 7;

        let _: complot::LogLog = (
            segments_piston_psd
                .frequency()
                .into_iter()
                .enumerate()
                .skip(1)
                .map(|(i, f)| {
                    (
                        f,
                        segments_piston_psd
                            .iter()
                            .skip(i)
                            .step_by(h)
                            .cloned()
                            .collect::<Vec<f64>>(),
                    )
                }),
            complot::complot!(
                "lom_segments-piston-psds.png",
                xlabel = "Frequency [Hz]",
                ylabel = "PSD [nm^2/Hz]"
            ),
        )
            .into();
        */
    }

    Ok(())
}
