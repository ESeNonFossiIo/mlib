#!/bin/sh

echo "===================================INIT=================================="
# Set tollerance for numdiff:
TOLLERANCE="1.0e-6"
IS_NUMDIFF=$(which numdiff)

if [ ${IS_NUMDIFF} ]
then
  DIFF="$(which numdiff) -V -r ${TOLLERANCE}"
else
  DIFF=$(which diff)
fi

${DIFF} $1 $2 2>&1
echo "===================================END==================================="
