# How To

This doc contains instructions for various tasks related to setting up and running the software.

# Setting Up the Environment

First, check that the environment setup has not been done before. If 
```bash
conda activate hotspot
```
succeeds, skip these steps.

## Python Dependencies
### Anaconda
If the Python environment/package manager [Anaconda](https://www.anaconda.com/) does not exist on the Raspberry Pi you're running this on, I recommend installing Miniconda [from here](https://docs.conda.io/en/master/miniconda.html). Get the installer for ARM processors (it has `aarch64` in the name) and follow the online instructions.

Once that is done, we are ready to set up the `hotspot` environment. `conda` allows specifying the packages needed in a file with a `.yml` extension. This is done for you. Create the `hotspot` conda env with

```bash
conda env create -f hotspot.yml
```

It should install things like `numpy` and `matplotlib`, as well as drivers for the hardware, such as Adafruit's `adafruit-circuitpython-motorkit` library for driving the steppers, and the library for controlling the Hawkeye IR sources via the LabJack.

Once that is done, activate the env with 

```bash
conda activate hotspot
```

If you need to install something else, remember to update `hotspot.yml` by doing 

```bash
conda env export --from-history | tee hotspot.yml
```

If your `hotspot.yml` has been updated, and you need to update your env with the new `hotspot.yml`, do

```bash
conda env update --file hotspot.yml --prune
```

## Enable Raspberry Pi Hardware

The raspberry pi should have at least two [motor driver hat boards](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi). These are PCBs with onboard chips that talk to the raspberry pi on an I2C bus via the 2x20 header pins. They issue commands to the motor driver chips, which handle the delivery and timing of greater voltage and current than the raspberry pi is capable of on its own.

Follow the steps for [Enabling I2C communication](https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/installing-software#enable-i2c-1106864-2) from Adafruit.

# Test Fixture Setup

For testing the software and hardware together, we set up the raspberry pi and the steppers on an optics bench in Steward Observatory Lab 168. This allows us to affix the steppers to something a roughly known distance apart.

## Stepper Fixture

Steppers are attached to the optics bench via 3D-printed brackets. 1/4-20 socket head cap screws affix the bracket to the bench, and M3 screws affix the steppers to the brackets.

## Spools

The spools are each attached to the 5mm stepper motor shaft via one M3 setscrew. The fishing line is affixed to the each spool by wrapping it around the setscrew and screwing it in to the threaded recess on the spool circumference. Positive motor rotation is defined by convention to spin the shaft clockwise when viewed from the rear of the motor. Motors should be oriented relative to the cable such that a positive motor rotation produces a positive cable length change (i.e., cable is played out from the spool), and a negative motor rotation winds cable onto the spool.

## Power

### Motors
The motor driver board must be powered via its own power supply, since the raspberry pi cannot provide the requisite voltage or current. A lab power supply with 12V output is attached to the +/- screw terminal block on the motor driver hat. The motor controllers on the hat are designed to run with 5-12V.

### LabJack
The LabJack board also needs its own power supply to drive the voltage/current that is switched via the breakout board. A tunable lab power supply is attached to one of the screw terminals labeled "VS#," for "voltage source #," where # is one of the channels, 1-6. The voltage of this power supply will depend on what is hooked up to the switchable terminals. In this case, we are using LEDs to stand in for Hawkeye IR sources, so 3.3V is fine.