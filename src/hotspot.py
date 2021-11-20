# This file houses the argument parsing and program setup
import argparse
import constants as const
import executive as ex
import logging
import os


if __name__ == "__main__":
    # -------------------------------------------------------------------------
    # ARGUMENT PARSING
    # -------------------------------------------------------------------------
    parser = argparse.ArgumentParser(description='Cable-driven parallel robot for positioning and driving IR sources to make beam maps of surfaces in weird places.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('geometry_file',
        type=str,
        help='(str) Choose the file that defines the corner point locations in the mirror coordinate frame and the size of the IR source raft.'
    )
    parser.add_argument('command_profile',
        type=str,
        help='(str) Choose the file that defines the sequence of locations and IR source actions to take.'
    )
    parser.add_argument('--loglevel',
        '-L',
        default='INFO',
        type=str,
        choices=['INFO', 'WARNING', 'DEBUG', 'ERROR', 'CRITICAL'],
        dest='loglevel',
        help='Python logging module loglevel.'
    )

    args = parser.parse_args()

    logging.basicConfig(level=args.loglevel)
    # Start up executive
    executive = ex.Executive(os.path.abspath(args.geometry_file))
    executive.run(os.path.abspath(args.command_profile))