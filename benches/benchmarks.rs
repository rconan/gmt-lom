use criterion::{criterion_group, criterion_main, Criterion};
use gmt_lom::LOM;
// use std::hint::black_box;

fn lom_tiptilt(c: &mut Criterion) {
    let lom = LOM::builder().build().unwrap();
    c.bench_function("LOM Tip-Tilt", |b| b.iter(|| lom.tiptilt()));
}
fn lom_segment_tiptilt(c: &mut Criterion) {
    let lom = LOM::builder().build().unwrap();
    c.bench_function("LOM Segment Tip-Tilt", |b| b.iter(|| lom.segment_tiptilt()));
}
fn lom_segment_piston(c: &mut Criterion) {
    let lom = LOM::builder().build().unwrap();
    c.bench_function("LOM Segment Piston", |b| b.iter(|| lom.segment_piston()));
}
fn lom_segment_wfe_rms(c: &mut Criterion) {
    let lom = LOM::builder().build().unwrap();
    c.bench_function("LOM Segment wfe_rms", |b| {
        b.iter(|| lom.segment_wfe_rms::<0>())
    });
}
fn lom_wavefront(c: &mut Criterion) {
    let lom = LOM::builder().build().unwrap();
    c.bench_function("LOM ", |b| b.iter(|| lom.wavefront()));
}

criterion_group!(
    benches,
    lom_tiptilt,
    lom_segment_tiptilt,
    lom_segment_piston,
    lom_segment_wfe_rms,
    lom_wavefront
);
criterion_main!(benches);
