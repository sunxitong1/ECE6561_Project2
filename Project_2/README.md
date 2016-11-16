## Project Summary

This project is meant to drive two DC-motors controlled by PWMs
based on a planned trajectory.

## Peripherals Exercised

* `Board_LED0`  - Blinks to indicate RTOS still running.
* `Board_PWM0`  - Operates the first of two motors
* `Board_PWM1`  - Operates the second of two motors

## Tasks

* `heartBeatFxn`         - Indicates board still alive
* `tMotorControl`        - Manages motor outputs
* `tSensorSuite`         - Handles input data from sensors
* `tTrajectoryPlanner`   - Plans trajectories and updates motor control

## Usage

* The application blinks `Board_LED0` using the `heartBeatFxn`
* The application controls the two PWM outputs to control Lego
NXT motors using a biased differential steering control scheme.

## RTOS Configuration Details

This examples is the same as the __Empty_Minimal__ example except many
development and debug features are enabled. For example:

* Logging is enabled
* Assert checking is enabled
* Kernel Idle task
* Stack overflow checking
* Default kernel heap is present

> Please refer to the __Memory Footprint Reduction__ section in the
> TI-RTOS User Guide *spruhd4.pdf* for a complete and detailed list of the
> differences between the empty minimal and empty projects.

## References
* For GNU and IAR users, please read the following website for details
  about enabling [semi-hosting](http://processors.wiki.ti.com/index.php/TI-RTOS_Examples_SemiHosting)
  in order to view console output.

* For more help, search either the SYS/BIOS User Guide or the TI-RTOS
  Getting Started Guide within your TI-RTOS installation.