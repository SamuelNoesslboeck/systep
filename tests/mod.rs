#![allow(non_snake_case)]

use syact::{AdvancedActuator, SyncActuator};
use systep::builder::{ComplexBuilder, StartStopBuilder};
use systep::{StepperConfig, StepperController, StepperData, StepperMotor};
use syunit::metric::{KgMeter2, NewtonMeters};
use syunit::{Direction, Factor, RadPerSecond3, Radians, Seconds};

pub const TEST_DATA : StepperData = StepperData {
    current_rated: 1.0,
    coil_inductance: 0.004,
    inertia_motor: KgMeter2(0.000_01),

    steps_per_rev: 200,
    coil_resistance: 4.0,
    torque_rated: NewtonMeters(0.40)
}; 

#[derive(Debug)]
pub struct SimulatedCtrl {
    pub step_list : Vec<Seconds>,
    pub dir_list : Vec<(usize, Direction)>,

    __dir : Direction
}

impl SimulatedCtrl {
    pub fn new() -> Self {
        Self {
            step_list: Vec::new(),
            dir_list: Vec::new(),

            __dir: Default::default()
        }
    }
}

impl StepperController for SimulatedCtrl {
    fn step(&mut self) -> Result<(), syact::ActuatorError<syunit::Rotary>> {
        Ok(())
    }

    fn dir(&self) -> syunit::Direction {
        self.__dir
    }

    fn set_dir(&mut self, dir : syunit::Direction) -> Result<(), syact::ActuatorError<syunit::Rotary>> {
        self.__dir = dir;
        self.dir_list.push((self.step_list.len(), dir));
        Ok(())
    }

    fn data(&self) -> &systep::ControllerData {
        todo!()
    }

    fn delay(&mut self, time : std::time::Duration) {
        self.step_list.push(Seconds(time.as_secs_f32()));
    }
}

pub type StartStopStepper = StepperMotor<StartStopBuilder, SimulatedCtrl>;
pub type ComplexStepper = StepperMotor<ComplexBuilder, SimulatedCtrl>;

#[test]
fn test__start_stop_builder() {
    let mut motor = StartStopStepper::new_advanced(
        SimulatedCtrl::new(), TEST_DATA, StepperConfig::VOLT12_NO_OVERLOAD
    ).unwrap();

    motor.drive_rel(Radians(1.0), Factor::MAX).unwrap();

    dbg!(motor.builder.velocity_possible());

    dbg!(motor.ctrl);
}

#[test]
fn test__complex_builder() {
    let mut motor = ComplexStepper::new_advanced(
        SimulatedCtrl::new(), TEST_DATA, StepperConfig::VOLT12_NO_OVERLOAD
    ).unwrap();

    // motor.set_microsteps(MicroSteps::HALF).unwrap();
    // motor.apply_gen_force(NewtonMeters(0.1)).unwrap();
    // motor.apply_inertia(KgMeter2(0.001)).unwrap();
    motor.set_jolt_max(Some(RadPerSecond3(10_000.0))).unwrap();
    motor.drive_rel(Radians(1.0), Factor::MAX).unwrap();

    dbg!(motor.builder.velocity_cap());
    dbg!(motor.builder.velocity_possible());

    dbg!(motor.builder);
    dbg!(motor.ctrl);
}