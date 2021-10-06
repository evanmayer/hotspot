# Software Organization

## I/O Requirements
### Inputs
- mode
    - home
    - jog
    - command profile
    - stare
- cable endpoint locations
- raft attachment point offsets
- command profiles
    - position sequence
    - linear velocity sequence
    - Hawkeye diode flash sequence?

### Outputs
- current mode
- motor steps to move
- motor steps per second
- current cmd
    - steps
    - steps / sec
- current cmd move progress (%)
- current sequence progress (%)
- estimated position
- move/sequence visualization

## Modules
### Control Algorithm
- calculate wire lengths given a commanded position
    - error check to ensure pos cmd is in surface's convex hull
- calculate change in wire lengths and dl/dt given the allowed time

### Hardware Interfaces (HWIF)
- compute commands for each motor, given change in wire lengths and dl/dt
- compute Hawkeye output commands?

### I/O Handling
- ingest configuration file to set physical config for a given mirror
- ingest command profile file into sequence of modes/commands
- handle human input
    - allow human to change modes
    - allow human to interrupt operation
    - translate jog commands into length-1 sequence of a position command
- output algorithm and HWIF telemetry (TM) to a file
- visualization

## Processor Requirements
Main program handles cmdline parsing, then event/state machine loop. Spawns:
- subprocess 0: 
    - human input, motor 0, motor 1, motor 2, motor 3
    - motor cmds may need to be async, depending on library/hwif speed
- subprocess 1: 
    - thread 0: TM output
    - thread 1: viz
