#![crate_name = "systep"]

use core::ops::{Div, Mul};

use syact::{ActuatorError, SyncActuator, RatioActuatorParent};
use syunit::*;

// Using diffent global allocator
extern crate alloc;

// ####################
// #    SUBMODULES    #
// ####################
    pub mod builder;

    mod ctrl;
    pub use ctrl::*;

    mod data;
    pub use data::*;

    mod motor;
    pub use motor::StepperMotor;
// 

// ################################
// #    StepperActuator-Traits    #
// ################################
    /// A component based on a stepper motor
    pub trait StepperActuator<U : UnitSet = Rotary> : SyncActuator<U> {
        // Microstepping
            /// The amount of microsteps in a full step
            fn microsteps(&self) -> MicroSteps;

            /// Set the amount of microsteps in a full step
            fn set_microsteps(&mut self, micro : MicroSteps) -> Result<(), ActuatorError<U>>;
        //

        // Steps
            /// The angular distance of a step considering microstepping
            fn step_dist(&self) -> U::Distance;
        // 
    }    

    // Automatic implementation of StepperActuator for any RatioActuatorParent
    impl<T : RatioActuatorParent> StepperActuator<T::Input> for T
    where
        T::Child : StepperActuator<T::Output>,

        <T::Input as UnitSet>::Time : From<<T::Output as UnitSet>::Time>,

        <T::Input as UnitSet>::Position : Div<T::Ratio, Output = <T::Output as UnitSet>::Position>,
        <T::Input as UnitSet>::Velocity : Div<T::Ratio, Output = <T::Output as UnitSet>::Velocity>,
        <T::Input as UnitSet>::Acceleration : Div<T::Ratio, Output = <T::Output as UnitSet>::Acceleration>,
        <T::Input as UnitSet>::Jolt : Div<T::Ratio, Output = <T::Output as UnitSet>::Jolt>,
        <T::Input as UnitSet>::Force : Mul<T::Ratio, Output = <T::Output as UnitSet>::Force>,
        <T::Input as UnitSet>::Inertia : InertiaUnit<T::Ratio, Reduced = <T::Output as UnitSet>::Inertia>,

        <T::Output as UnitSet>::Position : Mul<T::Ratio, Output = <T::Input as UnitSet>::Position>,
        <T::Output as UnitSet>::Distance : Mul<T::Ratio, Output = <T::Input as UnitSet>::Distance>,
        <T::Output as UnitSet>::Velocity : Mul<T::Ratio, Output = <T::Input as UnitSet>::Velocity>,
        <T::Output as UnitSet>::Acceleration : Mul<T::Ratio, Output = <T::Input as UnitSet>::Acceleration>,
        <T::Output as UnitSet>::Jolt : Mul<T::Ratio, Output = <T::Input as UnitSet>::Jolt>,
        <T::Output as UnitSet>::Force : Div<T::Ratio, Output = <T::Input as UnitSet>::Force>
    {
        // Microsteps
            fn microsteps(&self) -> MicroSteps {
                self.child().microsteps()
            }

            fn set_microsteps(&mut self, micro : MicroSteps) -> Result<(), ActuatorError<T::Input>> {
                self.child_mut().set_microsteps(micro)
                    .map_err(|err| self.error_for_parent(err))
            }
        // 

        fn step_dist(&self) -> <T::Input as UnitSet>::Distance {
            self.dist_for_parent(self.child().step_dist())
        }
    }
// 