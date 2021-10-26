import numpy as np
import matplotlib.pyplot as plt
from labjack import ljm as lj
import time
import threading
import sys
import os

# Simple dictionary to map relay number to modbus register on the LJ

relay_dict = {
    '1': 2008,'2': 2009,'3': 2010,'4': 2011,'5': 2012,'6': 2013,
    '7': 2014,'8': 2015,'9': 2016,'10': 2017,'11': 2018,'12': 2019
}

#############################################################################
# Function to try and open a connection the a LabJack, also a wrapper       #
# This function will catch the connection errors and try as long as asked   #
#############################################################################

def try_open(model,mode):
    while(1):
        try:
            name = lj.openS(model,mode)
            break
        except lj.LJMError as err:
            print('Error opening LJ connection')
            print(getattr(err,'errorString'))
            answer = input('Try again? Y/N: ')
            if answer == 'Y':
                time.sleep(1)
            else:
                return -1
    return name


#############################################################################
# Functions that write on,off, or commandable to the target LabJack         #
# These are basically just wrappers around the lj library that catch errors #
#############################################################################

def write_up(handle, addr):
    try:
        lj.eWriteAddress(handle, addr, 0,1)
    except lj.LJMError as err:
        print("Error in write to LJ, specific error is:")
        print(getattr(err,'errorCode'))
        print(getattr(err,'errorString'))
        return False
    return True

def write_down(handle, addr):
    try:
        lj.eWriteAddress(handle, addr, 0,0)
    except lj.LJMError as err:
        print("Error in write to LJ, specific error is:")
        print(getattr(err,'errorCode'))
        print(getattr(err,'errorString'))
        return False
    return True

def write_value(handle, addr, value):
    try:
        lj.eWriteAddress(handle, addr, 0,value)
    except lj.LJMError as err:
        print("Error in write to LJ, specific error is:")
        print(getattr(err,'errorCode'))
        print(getattr(err,'errorString'))
        return False
    return True

#############################################################################
# Wrappers around the write up, down, and value functions.                  #
# For use being called as threads, the error catching is useless as         #
# Interrupts are not passed to threads.                                     #
#############################################################################

def threaded_on(handle,target):
    written = 0
    while(1):
        try:
            if written == False:
                written = write_up(handle,target)
                time.sleep(1)
            else:
                break
        except KeyboardInterrupt:
            answer = input('Do you want to interrupt? ')
            if answer.lower() == 'y':
                break
    return

def threaded_off(handle,target):
    written = 0
    while(1):
        try:
            if written == False:
                written = write_down(handle,target)
                time.sleep(1)
            else:
                break
        except KeyboardInterrupt:
            answer = input('Do you want to interrupt? ')
            if answer.lower() == 'y':
                break
    return

def threaded_write(handle,target,value):
    written = 0
    while(1):
        try:
            if written == False:
                written = write_value(handle,target,value)
                if written == False:
                    time.sleep(1)
                else:
                    break
            else:
                break
        except KeyboardInterrupt:
            answer = input('Do you want to interrupt? ')
            if answer.lower() == 'y':
                break
    return

#############################################################################
# Pair of functions for spawning comm threads with the LJs.                 #
# The first function spawns threads and passes the state the relays will    #
# need to take on, while the second shuts all relays off                    #
#############################################################################


def thread_spawn_all(handle, states):
    for key in relay_dict.keys():
        thread = threading.Thread(target=threaded_write, args=(handle,relay_dict[key], states[int(key) - 1]),daemon=True)
        thread.start()
    return

def thread_spawn_all_off(handle):
    for key in relay_dict.keys():
        thread = threading.Thread(target=threaded_write, args=(handle,relay_dict[key], 0),daemon=True)
        thread.start()
    return
    
#############################################################################
# Function to be called in a thread that handles the labjack instructions.  #
# The first step is to process in a list or a text file of instructions     #
# Next, the function executes the first item in the list and waits for      #
# a read signal to be reset to 1 to proceed to the next step. At the end    #
# it sends the stop signal, which nominally can close a loop or something.  #
#############################################################################


def labjack_ingest_and_do(handle,instructions=None):
    # currently just saving my instruction files in CWD
    global read, stop
    path = os.getcwd()+'/'
    print(path)
    has_written = 0
    instruction_counter = 0
    if instructions is not None and isinstance(instructions,str):
        print('Loading instructions from file at '+path+instructions)
        instructions = np.loadtxt(path+instructions)
    elif instructions is not None:
        print('Passed array of instructions rather than file')
    else:
        print('No instructions passed, returning')
        stop = 1
        return
    while stop is not 1 and instruction_counter < len(instructions):
        if read is 1:
            # print('\rReading instruction line '+str(instruction_counter+1)+' of '+str(len(instructions)),end='',flush=True)
            print('Reading instruction line '+str(instruction_counter+1)+' of '+str(len(instructions)),end='\n',flush=True)
            read = 0
            has_written = 0
            states = instructions[instruction_counter]
            instruction_counter += 1
        if read is not 1:
            if has_written is 0:
                has_written = 1
                thread_spawn_all(handle,states)
            #########################
            # Testing Purposes Only #
            #########################
            # time.sleep(0.2)
            # read = 1
    if instruction_counter is len(instructions):
        print('Exiting LJ instruction thread and turning off LEDs')
        thread_spawn_all_off(handle)
        stop = 1
        return

# testing portion of the program, this should act similarly to a ~ main ~ routine

# start by opening a connection
handle = try_open('T7','USB')

# I'll generate some random instructions here
length = int(input('How many instructions? '))
random_instructions = np.random.randint(0,2,[length,12])

# for good measure, we'll save them and use the load style
path = os.getcwd()+'/'
np.savetxt(os.getcwd()+'/rand_inst.txt',random_instructions)

# now we'll set up the loop and start the thread
read = 1
stop = 0
time_sleep = float(input('How long to sleep? '))
lj_command = threading.Thread(target=labjack_ingest_and_do,args=(handle,'rand_inst.txt'),daemon=True)
lj_command.start()
while stop is not 1:
    time.sleep(time_sleep)
    read = 1
print('Closing connection to LabJack')
lj.close(handle)