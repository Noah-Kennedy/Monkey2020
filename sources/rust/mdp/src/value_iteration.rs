// use crate::State;
//
// pub trait ValueIterationState: State {
//     /// Read the intrinsic, immutable value of the state.
//     fn intrinsic_value(&self) -> f32;
//
//     /// The extrinsic value of the state that is estimated.
//     /// NAN if terminal.
//     fn extrinsic_value(&self) -> f32;
//
//     fn total_value(&self) -> f32 {
//         self.extrinsic_value() + self.extrinsic_value()
//     }
//
//     /// Set the extrinsic value of the state that is estimated.
//     /// Return the old value.
//     /// NAN if terminal.
//     fn write_extrinsic_value(&mut self, value: f32) -> f32;
// }
//
