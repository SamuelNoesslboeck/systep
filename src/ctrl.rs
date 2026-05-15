use core::time::Duration;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

use syact::ActuatorError;
use syunit::*;

use crate::ControllerData;

/// A controller for the logics of a stepper motor
pub trait StepperController {
    /// Does a single step
    fn step(&mut self) -> Result<(), ActuatorError<Rotary>>;

    /// The movement direction of the motor
    fn dir(&self) -> Direction;

    /// Sets the direction of the motor
    fn set_dir(&mut self, dir : Direction) -> Result<(), ActuatorError<Rotary>>;

    /// Getter for the controller data used
    fn data(&self) -> &ControllerData;

    fn delay(&mut self, time : Duration);
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

            _dir: Direction::CW         // TODO: Implement pin setup
        }
    }

    /// The time that the pulse will be held for the stepper signal
    pub fn pulse_time(&self) -> Seconds {
        0.5 / self._data.max_freq 
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
            self.pin_dir.set_high()
                .map_err(|_| ActuatorError::PinError)?;    
        } else {
            self.pin_dir.set_low()
                .map_err(|_| ActuatorError::PinError)?;  
        }

        self._dir = dir;
        Ok(())
    }
    
    fn step(&mut self) -> Result<(), ActuatorError<Rotary>> {
        self.pin_pul.set_high()
                .map_err(|_| ActuatorError::PinError)?;    
        self.timer.delay_us((self.pulse_time() * 1_000_000.0).0 as u32);  
        self.pin_pul.set_low()
            .map_err(|_| ActuatorError::PinError)?;    

        Ok(())
    }

    fn delay(&mut self, time : Duration) {
        self.timer.delay_us(
            time.as_micros() as u32     // TODO: High accuracy not required here! Still improvments to be made tho!
        );
    }
}