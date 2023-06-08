/*!
# Linear Optical Model client

The module implements the client interface for the [GMT LOM](https://docs.rs/gmt-lom)

The location of the LOM sensitivities matrices is given by the `LOM` environment variable

*The client is enabled with the `lom` feature.*

# Example

```
use dos_actors::clients::lom::*;
use dos_actors::prelude::*;
let lom: Actor<_> = lom::LOM::builder().build().unwrap().into();
```

*/
use crate::LOM;
use gmt_dos_clients::interface::{Data, Read, Update, Write, UID};
use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use std::convert::AsMut;

impl Update for LOM {}

impl Read<M1RigidBodyMotions> for LOM {
    fn read(&mut self, data: Data<M1RigidBodyMotions>) {
        self.rbm
            .as_mut()
            .column_mut(0)
            .iter_mut()
            .take(42)
            .zip(&**data)
            .for_each(|(rbm, val)| *rbm = *val);
    }
}

impl Read<M2RigidBodyMotions> for LOM {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        //dbg!((**data).iter().sum::<f64>() * 1e6);
        self.rbm
            .as_mut()
            .column_mut(0)
            .iter_mut()
            .skip(42)
            .zip(&**data)
            .for_each(|(rbm, val)| *rbm = *val);
    }
}

/// Tip and tilt in the GMT focal plane
#[derive(UID)]
pub enum TipTilt {}
impl Write<TipTilt> for LOM {
    fn write(&mut self) -> Option<Data<TipTilt>> {
        Some(Data::new((*self.tiptilt()).clone()))
    }
}
/// Segment tip and tilt in the GMT focal plane
#[derive(UID)]
pub enum SegmentTipTilt {}
impl Write<SegmentTipTilt> for LOM {
    fn write(&mut self) -> Option<Data<SegmentTipTilt>> {
        Some(Data::new((*self.segment_tiptilt()).clone()))
    }
}
#[cfg(feature = "fsm")]
impl Write<fsm::TTFB> for LOM {
    fn write(&mut self) -> Option<Data<fsm::TTFB>> {
        Some(Data::new((*self.segment_tiptilt()).clone()))
    }
}
/// Segment piston in the GMT exit pupil
#[derive(UID)]
pub enum SegmentPiston {}
impl Write<SegmentPiston> for LOM {
    fn write(&mut self) -> Option<Data<SegmentPiston>> {
        Some(Data::new((*self.segment_tiptilt()).clone()))
    }
}
