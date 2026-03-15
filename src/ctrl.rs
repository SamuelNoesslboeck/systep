use core::time::Duration;

use embedded_hal::digital::OutputPin;

use syact::ActuatorError;
use syunit::*;

use crate::ControllerData;

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Initializes a step with the given `time`
    fn step(&mut self, time : Seconds) -> Result<(), ActuatorError<Rotary>>;

    /// The movement direction of the motor
    fn dir(&self) -> Direction;

    /// Sets the direction of the motor
    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError<Rotary>>;

    fn data(&self) -> &ControllerData;
}

#[derive(Clone, Debug)]
pub struct GenericPulseCtrl<D : OutputPin, P : OutputPin> {
    _data : ControllerData,

    pub pin_dir : D,
    pub pin_pul : P,

    _dir : Direction
}

impl<D : OutputPin, P : OutputPin> GenericPulseCtrl<D, P> {
    pub fn new(data : ControllerData, pin_dir : D, pin_pul : P) -> Self {
        Self {
            _data: data,

            pin_dir,
            pin_pul,

            _dir: Direction::default()
        }
    }
}

impl<D : OutputPin, P : OutputPin> StepperController for GenericPulseCtrl<D, P> {
    fn dir(&self) -> Direction {
        self._dir
    }

    fn data(&self) -> &ControllerData {
        &self._data
    }

    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError<Rotary>> {
        if dir.as_bool() {
            self.pin_dir.set_high().unwrap();   // TODO: Remove
        } else {
            self.pin_dir.set_low().unwrap();    // TODO: Remove
        }

        self._dir = dir;
        Ok(())
    }
    
    fn step(&mut self, time : Seconds) -> Result<(), ActuatorError<Rotary>> {
        let half_time : Duration = (time / 2.0).into();

        self.pin_pul.set_high().unwrap();       // TODO: Improve pulse algorithm
        std::thread::sleep(half_time);
        self.pin_pul.set_low().unwrap();
        std::thread::sleep(half_time);

        Ok(())
    }
}