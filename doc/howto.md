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

<img src="img/spool_side_view.jpg" alt="drawing" width="200"/> <img src="img/spool_bench_mounted.jpg" alt="drawing" width="200"/>

## Spools

The spools are each attached to the 5mm stepper motor shaft via one M3 setscrew. The fishing line is affixed to the each spool by wrapping it around the setscrew and screwing it in to the threaded recess on the spool circumference. Positive motor rotation is defined by convention to spin the shaft clockwise when viewed from the rear of the motor. Motors should be oriented relative to the cable such that a positive motor rotation produces a positive cable length change (i.e., cable is played out from the spool), and a negative motor rotation winds cable onto the spool.


## Power

### Motors
The motor driver board must be powered via its own power supply, since the raspberry pi cannot provide the requisite voltage or current. A lab power supply with 12V output is attached to the +/- screw terminal block on the motor driver hat. The motor controllers on the hat are designed to run with 5-12V.

### LabJack
The LabJack board also needs its own power supply to drive the voltage/current that is switched via the breakout board. A tunable lab power supply is attached to one of the screw terminals labeled "VS#," for "voltage source #," where # is one of the channels, 1-6. The voltage of this power supply will depend on what is hooked up to the switchable terminals. In this case, we are using LEDs to stand in for Hawkeye IR sources, so 3.3V is fine.

## Communication

### Raspberry Pi
You can log in to the raspberry pi via `ssh`. In order for you computer to "see" the raspberry pi, though, they must be on the same network. This can be accomplished a few ways (or order of ease of use):
1. By connecting both computers to a router or network switch that can assigns each connected device an IP address automatically. Wired is easier than [wireless](https://www.raspberrypi.com/documentation/computers/configuration.html#setting-up-a-headless-raspberry-pi).
1. By connecting directly to the pi via an Ethernet patch cable and setting up a [link-local](https://en.wikipedia.org/wiki/Link-local_address) connection
1. By connecting directly to the pi via an Ethernet patch cable and assigning static IP addresses to each host.

The first option is the easiest, but depends on having access to an exisitng network, so limits your connectivity options. If you have access to you are on a Linux machine, the second option is about as easy and more flexible, in my opinion.

 The hostname and password are printed on the bottom of the white plastic case. Once connection is sorted out, ping the pi to make sure:
 
 ```ping timepi.local```
 
 The ssh command goes like this:

 ```ssh -X pi@timepi.local```

 `-X` allows X-forwarding, in case a graphical application (like plotting) is invoked. You will be prompted for a password, which you can find printed on the bottom of the white plastic raspberry pi case.

### Motors

Motors should be connected to the screw terminals of the pi's stepper hat like this:

<img src="img/stepper_wiring.jpg" alt="drawing" width="200"/>

Getting the order right ensures the stepper's coils are energized in the correct sequence to get it to rotate.

On the lab bench in SO168, I wired them up like this:

```
'sw': kit0.stepper1,
'ne': kit1.stepper1,
'nw': kit1.stepper2,
'se': kit0.stepper2
```

where kit0 is the bottom driver board, and stepper1 uses M1 & M2 terminal block on the driver board. Then I made sure that the roles coded in `Executive.__init__()` matched how the motors and drive cables were configured.