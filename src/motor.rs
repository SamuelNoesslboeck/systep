use alloc::boxed::Box;
use alloc::vec::Vec;

use syunit::*;
use syunit::metric::*;

use syact::{ActuatorError, AdvancedActuator, DefinedActuator, InterruptReason, Interruptible, Interruptor, SyncActuator};

use crate::{MicroSteps, StepperActuator, StepperConfig, StepperController, StepperData};
use crate::builder::{AdvancedStepperBuilder, SimpleStepperBuilder, StepperBuilder, StepperDriveMode};

/// A stepper motor
/// 
/// Controlled by two pins, one giving information about the direction, the other about the step signal (PWM)
pub struct StepperMotor<B : StepperBuilder, C : StepperController> {
    pub builder : B,
    pub ctrl : C, 

    moving : bool,    
    _pos : PositionRad,

    // Limits
    _limit_min : Option<PositionRad>,
    _limit_max : Option<PositionRad>,

    // Interrupters
    interruptors : Vec<Box<dyn Interruptor + Send>>,
    _intr_reason : Option<InterruptReason>,
}

// Inits
impl<B : StepperBuilder, C : StepperController> StepperMotor<B, C> {   
    /// ######################################
    /// #    StepperMotor::handle_builder    #
    /// ######################################
    ///
    /// Main driving algorithm for stepper motors, handles the builder until no nodes are left anymore
    pub async fn handle_builder(&mut self) -> Result<(), ActuatorError> {
        // Update the movement variable
        self.moving = true;
        
        // Iterate through the builder until no nodes are left
        while let Some(node) = self.builder.next() {
            // Get the current direction of the motor (builder)
            let direction = self.builder.direction();
            // Get the current drive mode of the motor (builder)
            let drive_mode = self.builder.drive_mode();

            // Check all interruptors if the motor is not stopping already
            if *drive_mode != StepperDriveMode::Stop {
                for intr in self.interruptors.iter_mut() {
                    // Check if the direction is right
                    if let Some(i_dir) = intr.dir() {
                        if i_dir != direction {
                            // If the interruptors checking-direction does not match the current direction, 
                            //     the loop skips to the next interruptor
                            continue;
                        }
                    }

                    // Checks if the interruptor has been triggered
                    if let Some(reason) = intr.check() {
                        intr.set_temp_dir(Some(direction));
                        self._intr_reason.replace(reason);
                        
                        self.builder.set_drive_mode(StepperDriveMode::Stop, &mut self.ctrl)?; 
                    } else {
                        // Clear temporary direction
                        intr.set_temp_dir(None);
                    }
                }
            }

            // Make step and return error if occured
            self.ctrl.step(node).await?;

            // Check if the pos value exeeds any limits, stop the movement if it does
            if direction.as_bool() {
                self._pos += self.builder.step_angle(); 

                // Stop the motor if the limit is exceeded
                if self.pos() > self.limit_max().unwrap_or(PositionRad::INFINITY) {
                    self.builder.set_drive_mode(StepperDriveMode::Stop, &mut self.ctrl)?;
                } 
            } else {
                self._pos -= self.builder.step_angle(); 

                // Stop the motor if the limit is exceeded
                if self.pos() < self.limit_min().unwrap_or(PositionRad::NEG_INFINITY) {
                    self.builder.set_drive_mode(StepperDriveMode::Stop, &mut self.ctrl)?;
                } 
            }
        }

        // No movement anymore
        self.moving = false;

        Ok(())
    }

    /// Returns the current movement direction
    pub fn direction(&self) -> Direction {
        self.builder.direction()
    }
}

// #######################################
// #    SyncActuator - Implementation    #
// #######################################
    impl<B : StepperBuilder, C : StepperController> SyncActuator for StepperMotor<B, C> {
        // Position 
            #[inline]
            fn pos(&self) -> PositionRad {
                self._pos
            }   

            #[inline]
            fn overwrite_abs_pos(&mut self, pos : PositionRad) {
                self._pos = pos;
            }
        //

        // State
            fn is_moving(&self) -> bool {
                self.moving
            }
        // 

        // Velocity
            fn velocity(&self) -> RadPerSecond {
                self.builder.velocity()
            }

            #[inline]
            fn velocity_max(&self) -> Option<RadPerSecond> {
                self.builder.velocity_max()
            }

            #[inline]
            fn set_velocity_max(&mut self, velocity_opt : Option<RadPerSecond>) -> Result<(), ActuatorError> {
                self.builder.set_velocity_max(velocity_opt)?;
                Ok(())
            }
        //

        // Acceleration
            #[inline]
            fn acceleration_max(&self) -> Option<RadPerSecond2> {
                self.builder.acceleration_max()
            }

            fn set_acceleration_max(&mut self, acceleration_opt : Option<RadPerSecond2>) -> Result<(), ActuatorError> {
                self.builder.set_acceleration_max(acceleration_opt)?;
                Ok(())
            }
        //

        // Jolt
            fn jolt_max(&self) -> Option<RadPerSecond3> {
                self.builder.jolt_max()
            }

            fn set_jolt_max(&mut self, jolt_opt : Option<RadPerSecond3>) -> Result<(), ActuatorError> {
                self.builder.set_jolt_max(jolt_opt)?;
                Ok(())
            }
        //

        // Position limits
            #[inline]
            fn limit_max(&self) -> Option<PositionRad> {
                self._limit_max
            }

            #[inline]
            fn limit_min(&self) -> Option<PositionRad> {
                self._limit_min
            }
            
            #[inline]
            fn set_pos_limits(&mut self, min : Option<PositionRad>, max : Option<PositionRad>) {
                if let Some(min) = min {
                    self._limit_min = Some(min)
                }

                if let Some(max) = max {
                    self._limit_max = Some(max);
                }
            }

            #[inline]
            fn overwrite_pos_limits(&mut self, min : Option<PositionRad>, max : Option<PositionRad>) {
                self._limit_min = min;
                self._limit_max = max;
            }

            fn set_endpos(&mut self, overwrite_abs_pos : PositionRad) {
                self.overwrite_abs_pos(overwrite_abs_pos);

                let dir = self.direction().as_bool();
        
                self.set_pos_limits(
                    if dir { None } else { Some(overwrite_abs_pos) },
                    if dir { Some(overwrite_abs_pos) } else { None }
                )
            }
        //

        async fn drive_rel(&mut self, rel_dist : Radians, speed_f : Factor) -> Result<(), ActuatorError> {
            if !rel_dist.is_finite() {
                return Err(ActuatorError::InvaldRelativeDistance(rel_dist));
            }

            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(StepperDriveMode::FixedDistance(rel_dist, RadPerSecond::ZERO, speed_f), &mut self.ctrl)?;
            self.handle_builder().await
        }

        async fn drive_factor(&mut self, speed : Factor, direction : Direction) -> Result<(), ActuatorError> {
            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(StepperDriveMode::ConstFactor(speed, direction), &mut self.ctrl)?;
            self.handle_builder().await
        }
    
        async fn drive_speed(&mut self, speed : RadPerSecond) -> Result<(), ActuatorError> {
            // Set drive mode, return mapped error if one occurs
            self.builder.set_drive_mode(StepperDriveMode::ConstVelocity(speed), &mut self.ctrl)?;
            self.handle_builder().await
        }
    }
// 

// ###########################
// #    Builder dependent    #
// ###########################
    impl<B : SimpleStepperBuilder, C : StepperController> StepperMotor<B, C> {
        /// Creates a new stepper motor with the given controller `ctrl` 
        pub fn new_simple(ctrl : C) -> Result<Self, ActuatorError> {
            Ok(Self {
                builder: B::new()?,
                ctrl,

                moving: false,
                _pos: PositionRad::ZERO,

                _limit_min: None,
                _limit_max: None,

                interruptors : Vec::new(),
                _intr_reason: None
            })
        }
    }

    impl<B : AdvancedStepperBuilder, C : StepperController> StepperMotor<B, C> {
        /// Creates a new stepper motor with the given constants `consts` and configuration `config`
        pub fn new_advanced(ctrl : C, consts : StepperData, config : StepperConfig) -> Result<Self, ActuatorError> {
            Ok(Self {
                builder: B::new(consts, config)?,
                ctrl,

                moving: false,
                _pos: PositionRad::ZERO,

                _limit_min: None,
                _limit_max: None,

                interruptors : Vec::new(),
                _intr_reason: None
            })
        }
    }

    impl<B : AdvancedStepperBuilder, C : StepperController> AdvancedActuator for StepperMotor<B, C> {
        // Loads
            fn force_gen(&self) -> NewtonMeters {
                self.builder.vars().force_load_gen
            }

            fn force_dir(&self) -> NewtonMeters {
                self.builder.vars().force_load_dir
            }

            fn apply_gen_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError> {
                self.builder.apply_gen_force(force)?;
                Ok(())
            }

            fn apply_dir_force(&mut self, force : NewtonMeters) -> Result<(), ActuatorError> {
                self.builder.apply_dir_force(force)?;
                Ok(())
            }

            fn inertia(&self) -> KgMeter2 {
                self.builder.vars().inertia_load
            }

            #[inline(always)]
            fn apply_inertia(&mut self, inertia : KgMeter2) -> Result<(), ActuatorError> {
                self.builder.apply_inertia(inertia)?;
                Ok(())
            }
        //
    }

    
    impl<B : StepperBuilder, C : StepperController> StepperActuator for StepperMotor<B, C> {
        // Data
            fn microsteps(&self) -> MicroSteps {
                self.builder.microsteps()
            }

            fn set_microsteps(&mut self, microsteps : MicroSteps) -> Result<(), ActuatorError> {
                self.builder.set_microsteps(microsteps)
                
            }
        //

        fn step_dist(&self) -> Radians {
            self.builder.step_angle()
        }
    }
//

impl<B : StepperBuilder, C : StepperController> Interruptible for StepperMotor<B, C> {
    // Interruptors
        fn add_interruptor(&mut self, interruptor : Box<dyn Interruptor + Send>) {
            self.interruptors.push(interruptor);
        }

        fn intr_reason(&mut self) -> Option<InterruptReason> {
            // Return the value and replace it with `None`
            core::mem::replace(&mut self._intr_reason, None)
        }
    // 
}

impl<B : StepperBuilder, C : StepperController> DefinedActuator for StepperMotor<B, C> 
where
    B : DefinedActuator 
{
    fn ptp_time_for_distance(&self, abs_pos_0 : PositionRad, abs_pos_t : PositionRad) -> Seconds {
        self.builder.ptp_time_for_distance(abs_pos_0, abs_pos_t)
    }
}