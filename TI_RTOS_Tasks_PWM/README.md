## Example Summary

Sample application to control two on-board PWMs.

## Peripherals Exercised

* `Board_PWM0` - PWM instance used to control IO 2.6.
* `Board_PWM1` - PWM instance used to control IO 2.7

## Resources & Jumper Settings

> Please refer to the development board's specific __Settings and Resources__
section in the Getting Started Guide. For convenience, a short summary is also
shown below.

| Development board | Notes                                                  |
| ----------------- | ---------                                              |
| CC3200            | Close jumpers J2 and J3                                |
| DK-TM4C129X       |                                                        |
| EK-TM4C123GXL     |                                                        |
| EK-TM4C1294XL     |                                                        |
| EK-TM4C129EXL     |                                                        |
| MSP-EXP432P401R   |                                                        |

> Fields left blank have no specific settings for this example.

## Example Usage

* The example performs general initialization in `main()`.

## Application Design Details

This application uses two instances of the task, which performs the following actions:

1. Opens and initializes PWM driver objects.

2. Uses the PWM driver to change PWM modulation length.
