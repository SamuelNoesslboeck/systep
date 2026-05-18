use core::f32::consts::{E, PI};
use core::ops::Mul;
use core::str::FromStr;

use serde::{Serialize, Deserialize};
use syunit::metric::{KgMeter2, NewtonMeters};
use syunit::*;

use syact::data::ActuatorVars;

/// Microsteps used for stepper motors
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct MicroSteps(u8);

impl MicroSteps {
    /// Halfing the A-Phases, leading to twice as many steps
    pub const HALF_A : Self = Self(2);
    /// Halfing the B-Phases, leading to twice as many steps
    pub const HALF_B : Self = Self(2);
    /// Halfing both phases, leading to 4 microsteps
    pub const HALF : Self = Self(4);
    /// Quatering both phases, leading to 16 microsteps
    pub const QUATER : Self = Self(16);

    /// Get the representing `u8` value
    pub fn as_u8(self) -> u8 {
        self.0
    }
}

impl From<u8> for MicroSteps {
    fn from(value: u8) -> Self {
        if (value & value.wrapping_sub(1)) == 0 {   // Check if power of 2
            Self(value)
        } else {
            panic!("Number of microsteps must be a power of 2! ({} given)", value)
        }
    }
}

impl FromStr for MicroSteps {
    type Err = <u8 as FromStr>::Err;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self(u8::from_str(s)?))
    }
}

impl From<MicroSteps> for u8 {
    fn from(value: MicroSteps) -> Self {
        value.0
    }
}

impl Default for MicroSteps {
    fn default() -> Self {
        Self(1)
    }
}

impl Mul<MicroSteps> for u64 {
    type Output = u64;

    fn mul(self, rhs: MicroSteps) -> Self::Output {
        self * (rhs.as_u8() as u64)
    }
}

/// Stores data for generic components 
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StepperConfig {
    /// Supply voltage of the components in Volts
    pub voltage : f32,

    /// Maximum current of the stepper, can adjust torque
    pub max_current : Option<f32>
}

impl StepperConfig {
    /// The stepper is using 12 Volts and its rated current
    pub const VOLT12_NO_OVERLOAD : Self = Self {
        voltage: 12.0,
        max_current: None
    }; 

    /// The stepper is using 24 Volts and its rated current
    pub const VOLT24_NO_OVERLOAD : Self = Self {
        voltage: 24.0,
        max_current: None
    };

    /// The stepper is using 36 Volts and its rated current
    pub const VOLT36_NO_OVERLOAD : Self = Self {
        voltage: 36.0,
        max_current: None
    };

    /// The stepper is using 48 Volts and its rated current
    pub const VOLT48_NO_OVERLOAD : Self = Self {
        voltage: 48.0,
        max_current: None
    };

    /// Creates a new StepperConfig instance
    /// 
    /// ### Panic
    /// 
    /// Panics if a value smaller than or equal to `0.0` has been provided
    pub fn new(voltage : f32, overload_current : Option<f32>) -> Self {
        if voltage <= 0.0 {
            panic!("Voltage cannot be smaller than or equal to 0.0!");
        }

        Self { 
            voltage,
            max_current: overload_current
        }
    }
}

#[derive(Clone, Debug)]
pub struct ControllerData {
    /// Maximum frequency that can be driven with this driver
    pub max_freq : Hertz
}

/// A collection of the most relevant variables Unit stepper calculation 
/// ```
/// use syact::StepperConst;
///
/// // Create the data from an standard motor
/// let mut data = StepperConst::MOT_17HE15_1504S;
///
/// ``` 
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StepperData {
    /// Max current per phase (*Unit A*)
    pub current_rated : f32,
    /// Motor inductance (*Unit H*)
    pub coil_inductance : f32,
    /// Coil resistance (*Unit Ohm*)
    pub coil_resistance : f32,

    /// Step count per revolution (*Unit (1)*)
    pub steps_per_rev : u64,
    /// Rated torque (reached when the rated current is performed)
    pub torque_rated : NewtonMeters,
    /// Inhertia moment 
    pub inertia_motor : KgMeter2
}

impl StepperData {
    /* Data for specific motors */
        /// ### Stepper motor 17HE15-1504S
        /// Values for standard stepper motor
        pub const MOT_17HE15_1504S : Self = Self {
            current_rated: 1.5, 
            coil_inductance: 0.004, 
            coil_resistance: 2.3,
            steps_per_rev: 200, 
            torque_rated: NewtonMeters(0.42), 
            inertia_motor: KgMeter2(0.000_005_7)
        }; 

        /// ### Stepper motor 23HS45_4204S
        /// Values for standard stepper motor, see <https://www.omc-stepperonline.com/download/23HS45-4204S.pdf>
        pub const MOT_23HS45_4204S : Self = Self {
            current_rated: 3.8,
            coil_inductance: 0.0034,
            coil_resistance: 0.88,
            steps_per_rev: 400,
            torque_rated: NewtonMeters(3.0),
            inertia_motor: KgMeter2(0.000_068)
        };
    /**/

    /* Descriptive Constants */
        /// The charging/discharging constant
        #[inline(always)]
        pub const fn tau(&self) -> Seconds {
            Seconds(self.coil_inductance / self.coil_resistance)
        }

        /// The torque constant of the motor in Nm / Amperes
        pub const fn torque_const(&self) -> f32 {
            self.torque_rated.0 / self.current_rated
        }
    /**/

    /* Current */    
        /// Maximum current in amperes that can be configured with the given voltage
        /// 
        /// Note that this is **not the rated current** and it might damage your motor, be sure
        /// to configure your stepper driver to the [Self::rated_current]
        #[inline]
        pub const fn current_max(&self, voltage : f32) -> f32 {
            voltage / self.coil_resistance
        }

        /// Peak current in amperes that is reached during the movement process 
        pub fn current_peak(&self, mut velocity : RadPerSecond, config : &StepperConfig) -> f32 {
            velocity = velocity.abs();  // Direction does not matter to this formula

            if !velocity.is_finite() {
                panic!("Bad velocity ! {}", velocity);
            }
            
            if velocity == RadPerSecond::ZERO {
                return self.current_max(config.voltage);
            }

            let t_s = self.full_step_time(velocity);
            let e_pow = E.powf(-t_s / self.tau());

            self.current_max(config.voltage) * (1.0 - e_pow) / (1.0 + e_pow)
        }

        /// Average coil current for the given velocity and configuration
        pub fn current_avg(&self, mut velocity : RadPerSecond, config : &StepperConfig) -> f32 {
            // Code from current_peak copied for performance reasons, they share many variables!
            velocity = velocity.abs();  // Direction does not matter to this formula

            // Either the user has a special configured current ratio, or the rated one is used
            // => Unregulated cases are not handled, because they are unsafe and damage the motor
            let current_setting = config.max_current.unwrap_or(self.current_rated);

            if !velocity.is_finite() {
                panic!("Bad velocity ! {}", velocity);
            }
            
            if velocity == RadPerSecond::ZERO {
                return self.current_max(config.voltage).min(current_setting);
            }

            let t_s = self.full_step_time(velocity);
            let e_pow = E.powf(-t_s / self.tau());

            // Current values
            let current_max = self.current_max(config.voltage);
            let current_peak = current_max * (1.0 - e_pow) / (1.0 + e_pow);

            // Current is cut off by driver
            if current_peak > current_setting {
                // Overflow time (time the current would be higher than the set one)
                let t_ov = - self.tau() * ((current_max - current_setting) / (current_max + current_setting)).ln();
                let e_pow_mod = E.powf(-t_ov / self.tau());

                // Average coil current in non-overflow times
                let ic_avg = current_max - self.tau() / t_ov * (1.0 - e_pow_mod) * (current_max + current_setting);

                (ic_avg * t_ov + current_setting * (t_s - t_ov)) / t_s
            // No intervention from driver
            } else {
                current_max - self.tau() / t_s * (1.0 - e_pow) * (current_max + current_peak)
            }
        }
    /**/

    /* Torque */
        /// Maximum torque (stall torque) of the motor with the given current
        /// 
        /// ## Option
        ///
        /// Uses the **rated** current if the given option is `None`
        #[inline]
        pub fn torque_stall(&self, current_opt : Option<f32>) -> NewtonMeters {
            self.torque_rated * current_opt.unwrap_or(self.current_rated) / self.current_rated
        }

        /// Average torque for the given velocity with the given stepper configuration
        pub fn torque_avg(&self, velocity : RadPerSecond, config : &StepperConfig) -> NewtonMeters {
            NewtonMeters(self.torque_const() * self.current_avg(velocity, config))
        }
    // 

    // Acceleration
        /// Returns the maximum acceleration that can be reached in stall
        #[inline]
        pub fn acceleration_max_stall(&self, vars : &ActuatorVars, dir : Direction) -> Option<RadPerSecond2> {
            vars.force_after_load(self.torque_rated, dir).map(|f| f / vars.inertia_after_load(self.inertia_motor))
        }

        /// Returns the maximum acceleration that can be reached 
        #[inline]
        pub fn acceleration_max_for_velocity(&self, vars : &ActuatorVars, config : &StepperConfig, velocity : RadPerSecond, dir : Direction) -> Option<RadPerSecond2> {
            vars.force_after_load(self.torque_avg(velocity, config), dir).map(|f| f / vars.inertia_after_load(self.inertia_motor))
        }
    // 

    // Velocity
        /// Maximum speed for a stepper motor where it can be guarantied that it works properly
        #[inline(always)]
        pub fn velocity_max(&self, voltage : f32) -> RadPerSecond {
            RadPerSecond(4.0 * PI * voltage / (self.steps_per_rev as f32 * self.coil_inductance * self.current_rated))
        }

        /// Returns the start-stop-velocity for a stepper motor
        pub fn velocity_start_stop(&self, vars : &ActuatorVars, config : &StepperConfig, microsteps : MicroSteps) -> Option<RadPerSecond> {
            vars.force_after_load_lower(self.torque_stall(config.max_current)).map(|torque| {
                RadPerSecond((torque.0 / vars.inertia_after_load(self.inertia_motor).0 * core::f32::consts::PI / (self.steps_per_rev * microsteps) as f32).sqrt())
            })
        }
    // 

    /// Velocity for time per step [Unit 1/s]
    /// 
    /// # Panics
    /// 
    /// Panics if the given `step_time` is zero (`-0.0` included)
    /// 
    /// ```rust 
    /// use core::f32::consts::PI;
    /// 
    /// use syact::prelude::*;
    /// 
    /// let data = StepperConst::MOT_17HE15_1504S;
    /// 
    /// assert!((data.velocity(Seconds(1.0/200.0), MicroSteps::default()) - RadPerSecond(2.0 * PI)).abs() < RadPerSecond(0.001));     
    /// ```
    #[inline(always)]
    pub fn velocity(&self, step_time : Seconds, microsteps : MicroSteps) -> RadPerSecond {
        self.step_angle(microsteps) / step_time
    }

    // Step angles & times
        /// Get the angular distance of a step in radians, considering microstepping
        /// - `micro` is the amount of microsteps per full step
        #[inline(always)]
        pub fn step_angle(&self, microsteps : MicroSteps) -> Radians {
            Radians(self.full_step_angle().0 / microsteps.as_u8() as f32)
        }

        /// A full step angle of the motor, ignoring microstepping
        #[inline(always)]
        pub const fn full_step_angle(&self) -> Radians {
            Radians(2.0 * PI / self.steps_per_rev as f32)
        }

        /// Time per step for the given velocity 
        /// 
        /// # Unit
        /// 
        /// Returns the time in seconds
        /// 
        /// # Panics 
        /// 
        /// Panics if the given `velocity ` is zero 
        #[inline]
        pub fn step_time(&self, velocity  : RadPerSecond, microsteps : MicroSteps) -> Seconds {
            self.step_angle(microsteps) / velocity 
        }

        /// Time per full step at the given velocity velocity 
        /// 
        /// # Unit
        /// 
        /// Returns the time in seconds
        #[inline]
        pub fn full_step_time(&self, velocity  : RadPerSecond) -> Seconds {
            self.full_step_angle() / velocity 
        }
    // 

    // Steps & Angles - Conversions
        /// Converts the given angle `ang` into a absolute number of steps (always positive).
        #[inline(always)]
        pub fn steps_from_angle_abs(&self, angle : Radians, microsteps : MicroSteps) -> u64 {
            (angle.abs() / self.step_angle(microsteps)).round() as u64
        }   

        /// Converts the given angle `ang` into a number of steps
        #[inline(always)]
        pub fn steps_from_angle(&self, angle : Radians, microsteps : MicroSteps) -> i64 {
            (angle / self.step_angle(microsteps)).round() as i64
        }   

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn angle_from_steps_abs(&self, steps : u64, microsteps : MicroSteps) -> Radians {
            steps as f32 * self.step_angle(microsteps)
        }

        /// Converts the given number of steps into an angle
        #[inline(always)]
        pub fn angle_from_steps(&self, steps : i64, microsteps : MicroSteps) -> Radians {
            steps as f32 * self.step_angle(microsteps)
        }

        /// Rounds the given angle to the nearest step value
        #[inline(always)]
        pub fn round_angle_to_steps(&self, angle : Radians, microsteps : MicroSteps) -> Radians {
            self.angle_from_steps(self.steps_from_angle(angle, microsteps), microsteps)
        }

        // Comparision
        /// Checks wheither the given angle `ang` is in range (closes to) a given step count `steps`
        #[inline(always)]
        pub fn is_in_step_range(&self, steps : i64, angle : Radians, microsteps : MicroSteps) -> bool {
            self.steps_from_angle(angle, microsteps) == steps
        }
    //
}