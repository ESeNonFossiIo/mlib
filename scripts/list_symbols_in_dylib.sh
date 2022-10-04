#!/bin/bash

# Absolute path this script
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

nm -gU ${SCRIPTPATH}/../build/lib/lib_mlib.dylib
