use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

use syact::ActuatorError;
use syunit::*;

use crate::ControllerData;

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Initializes a step with the given `time`
    async fn step(&mut self, time : Seconds) -> Result<(), ActuatorError<Rotary>>;

    /// The movement direction of the motor
    fn dir(&self) -> Direction;

    /// Sets the direction of the motor
    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError<Rotary>>;

    /// Getter for the controller data used
    fn data(&self) -> &ControllerData;
}

#[derive(Clone, Debug)]
pub struct GenericPulseCtrl<D : OutputPin, P : OutputPin, T : DelayNs> {
    pub pin_dir : D,
    pub pin_pul : P,

    timer : T,

    _data : ControllerData,
    _dir : Direction
}

impl<D : OutputPin, P : OutputPin, T : DelayNs> GenericPulseCtrl<D, P, T> {
    pub fn new(data : ControllerData, pin_dir : D, pin_pul : P, timer : T) -> Self {
        Self {
            _data: data,

            pin_dir,
            pin_pul,

            timer,

            _dir: Direction::default()
        }
    }
}

impl<D : OutputPin, P : OutputPin, T : DelayNs> StepperController for GenericPulseCtrl<D, P, T> {
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
    
    async fn step(&mut self, time : Seconds) -> Result<(), ActuatorError<Rotary>> {
        let half_time : u32 = ((time / 2.0) * 1_000_000.0).0 as u32;

        self.pin_pul.set_high().unwrap();       // TODO: Improve pulse algorithm and delay implementation
        self.timer.delay_us(half_time);
        self.pin_pul.set_low().unwrap();
        self.timer.delay_us(half_time);

        Ok(())
    }
}