#![crate_name = "systep"]

use core::ops::{Div, Mul};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use atomic_float::AtomicF32;

use syact::{ActuatorError, SyncActuator, SyncActuatorState, RatioActuatorParent};
use syunit::*;

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

// ######################
// #    StepperState    #
// ######################
    /// The state of a stepper motor, whether it is driving etc.
    pub struct StepperState {
        /// Atomic `Radians`
        _abs_pos : AtomicF32,
        _moving : AtomicBool,

        should_halt : AtomicBool,
        should_interrupt : AtomicBool
    }

    impl StepperState {
        /// Creates a new `StepperState`
        pub fn new() -> Self {
            StepperState {
                _abs_pos: AtomicF32::new(Radians::ZERO.0),
                _moving: AtomicBool::new(false),

                should_halt : AtomicBool::new(false),
                should_interrupt : AtomicBool::new(false)
            }
        }
    }

    impl SyncActuatorState<Rotary> for StepperState {
        fn pos(&self) -> PositionRad {
            PositionRad(self._abs_pos.load(Relaxed))
        }

        fn moving(&self) -> bool {
            self._moving.load(Relaxed)
        }

        fn halt(&self) {
            self.should_halt.store(true, Relaxed);
        }

        fn interrupt(&self) {
            self.should_interrupt.store(true, Relaxed);
        }
    }
// 