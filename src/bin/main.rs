use gmt_lom::{Loader, LOM};

fn main() -> anyhow::Result<()> {
    let lom = LOM::builder()
        .load_rigid_body_motions(Loader::default().path("../wind2optics"))?
        .build()?;
    let tiptilt = lom.tiptilt();

    let _: complot::Plot = (
        lom.time()
            .iter()
            .zip(tiptilt.chunks(2))
            .map(|(&t, xy)| (t, xy.to_vec())),
        complot::complot!("lom.png", xlabel = "Time [s]", ylabel = "Tip-Tilt [arcsec]"),
    )
        .into();
    Ok(())
}
