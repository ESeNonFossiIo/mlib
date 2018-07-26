#!/bin/sh

# Set tollerance for numdiff:
TOLLERANCE="1.0e-6"
IS_NUMDIFF=$(which numdiff)

if [ ${IS_NUMDIFF} ]
then
  DIFF="$(which numdiff) -V -r ${TOLLERANCE}"
else
  DIFF=$(which diff)
fi

check_diff () {
  ${DIFF}  $1 $2 >/dev/null
  RESULT=$?
  echo "RESULT"=$RESULT
  if [ "$RESULT" -eq 0 ]
  then
    true
  else
    ${DIFF} $1 $2 2>&1 > $2.diff
    false
  fi
}

check_diff $1 $2
