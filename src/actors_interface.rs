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
use dos_actors::io::Read;
use dos_actors::{
    io::{Data, UniqueIdentifier, Write},
    Update, UID,
};
use std::convert::AsMut;
use std::sync::Arc;
use dos_clients_io::{M1RigidBodyMotions,M2RigidBodyMotions};

impl Update for LOM {}

impl Read<M1RigidBodyMotions> for LOM {
    fn read(&mut self, data: Arc<Data<M1RigidBodyMotions>>) {
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
    fn read(&mut self, data: Arc<Data<M2RigidBodyMotions>>) {
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
    fn write(&mut self) -> Option<Arc<Data<TipTilt>>> {
        Some(Arc::new(Data::new((*self.tiptilt()).clone())))
    }
}
/// Segment tip and tilt in the GMT focal plane
#[derive(UID)]
pub enum SegmentTipTilt {}
impl Write<SegmentTipTilt> for LOM {
    fn write(&mut self) -> Option<Arc<Data<SegmentTipTilt>>> {
        Some(Arc::new(Data::new((*self.segment_tiptilt()).clone())))
    }
}
#[cfg(feature = "fsm")]
impl Write<fsm::TTFB> for LOM {
    fn write(&mut self) -> Option<Arc<Data<fsm::TTFB>>> {
        Some(Arc::new(Data::new((*self.segment_tiptilt()).clone())))
    }
}
/// Segment piston in the GMT exit pupil
#[derive(UID)]
pub enum SegmentPiston {}
impl Write<SegmentPiston> for LOM {
    fn write(&mut self) -> Option<Arc<Data<SegmentPiston>>> {
        Some(Arc::new(Data::new((*self.segment_tiptilt()).clone())))
    }
}
