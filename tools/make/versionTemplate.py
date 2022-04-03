#!/usr/bin/env python

import sys
import subprocess
import os
from os import path
import re
import json
import argparse

version = {}

header = """/* This file is automatically generated by {0}!
 * Do not edit manually, any manual change will be overwritten.
 */
"""

def check_output(*args):
    """A wrapper for subprocess.check_output() to handle differences in python 2 and 3.
    Returns a string.
    """
    result = subprocess.check_output(*args)

    if isinstance(result, bytes):
        return result.decode('utf-8')
    else:
        return result


def extract_information_from_git(base):
    version['revision'] = "Dream Team"
    version['irevision0'] = "0x" + "FFFF"
    version['irevision1'] = "0x" + "FFFF"
    version['productionRelease'] = 'false'

    try:
        identify = "DreamTeam-V1"
        identify = identify.split('-')

        if len(identify) > 2:
            version['local_revision'] = identify[len(identify) - 2]
        else:
            version['local_revision'] = '0'

        version['tag'] = identify[0]
        for x in range(1, len(identify) - 2):
            version['tag'] += '-'
            version['tag'] += identify[x]
    except subprocess.CalledProcessError:
        # We are maybe running from a shallow tree
        version['local_revision'] = '0'
        version['tag'] = "NA"

    version['tag'] = version['tag'].strip()

    if version['local_revision'] != '0':
        version['tag'] = version['tag'] + ' +' + version['local_revision']

    version['branch'] = "Final Version"

    version['modified'] = 'false'


def extract_information_from_build_info_file():
    info_file = path.basename(path.abspath(path.dirname(__file__) +
                                           '/../../build_info.json'))
    if not path.exists(info_file):
        return False

    with open(info_file) as json_file:
        build_info = json.load(json_file)

    version['revision'] = 'NA'
    version['irevision0'] = "0x" + '0'
    version['irevision1'] = "0x" + '0'
    version['local_revision'] = 'NA'
    version['branch'] = 'NA'
    version['tag'] = build_info['tag']
    version['modified'] = 'false'
    version['productionRelease'] = 'true'

    return True


def print_version():
    if version['productionRelease'] == 'true':
        print("\033[1;31mProduction build {tag}\033[m".format(**version))
    else:
        status = "\033[1;32mCLEAN\033[m"
        if (version['modified'] == 'true'):
            status = "\033[1;31mMODIFIED\033[m"

        print("Build {local_revision}:{revision} ({tag}) {}".format(status,
                                                                    **version))
    print("Version extracted from {source}".format(**version))


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Source template", nargs="?", default=None)
    parser.add_argument("destination", help="Destination version file", nargs="?",default=None)
    parser.add_argument("--crazyflie-base", help="base folder of the crazyflie firmware",
                        action="store", default="./")
    parser.add_argument('--print-version', help="Print version and exit",
                        action="store_true")
    args = parser.parse_args()

    if extract_information_from_build_info_file():
        version['source'] = "build info file"
    else:
        version['source'] = "git"
        extract_information_from_git(args.crazyflie_base)

    if args.print_version:
        print_version()
        sys.exit(0)

    if not args.source or not args.destination:
        print("Usage:")
        print("  {0} <infile> <outfile>".format(sys.argv[0]))
        sys.exit(1)


    # Apply information to the file template
    infile = open(args.source, 'r')
    outfile = open(args.destination, 'w')

    outfile.write(header.format(sys.argv[0], args.source))
    outfile.write(infile.read().format(**version))

    infile.close()
    outfile.close()
