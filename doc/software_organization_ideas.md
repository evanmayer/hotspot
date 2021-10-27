# Software Organization

## I/O Requirements
### Inputs
- mode
    - cal (accept truth position input)
    - home (move to home pos)
    - jog (move a little bit on kbd input)
    - command profile (canned sequence)
    - stare (lights on, no movement)
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

### Executive
- ingest configuration file to set physical config for a given mirror
- run state machine:
    - ingest command profile file into sequence of modes/commands
    - kick off subprocesses/threads:
        - Control(subprocess):
            - feed commands to control algorithm
            - receive motor rotation commands from control algorithm
            - feed motor rotation commands to hardware interface
            - dump TM to an output buffer
        - handle human input (thread):
            - allow human to change modes: cal, home, jog, seq, stare
            - allow human to interrupt operation (E-stop/Ctrl-C)
            - translate jog commands into length-1 sequence of position commands
        - handle output:
            - pick TM out of buffer and write to file


### Control Algorithm
- calculate wire lengths given a commanded position
    - error check to ensure pos cmd is in surface's convex hull
- calculate change in wire lengths and dl/dt given the allowed time
- translate change in wire lengths into motor rotation commands

### Hardware Interfaces (hw if)
- feed motor rotation commands to motors
- compute Hawkeye output commands?

### I/O Handling
- output algorithm and HWIF telemetry (TM) to a file
- visualization

## Processor Requirements
Executive handles cmdline parsing, then starts state machine/event loop. Spawns:
- subprocess 0: 
    - spawns threads 0,1,2,3:  (motor 0, motor 1, motor 2, motor 3)
    - control alg runs at a set cadence
    - executive writes commands to motor buffers (or drains them to halt)
    - executive dumps control alg and motor TM to a buffer for output
    - spawns thread 5: listen for human input
        - keypresses buffered for handling by executive
- subprocess 1:
    - thread 0: TM output, then viz, on flip-flop
