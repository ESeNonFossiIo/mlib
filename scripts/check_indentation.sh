#!/bin/bash

sh ./scripts/indent
DIFF="$(git diff)"
BAR="========================================================================="
if [ -z "$DIFF" ]
then
  echo ${BAR}
  echo " Indentation OK"
  echo ${BAR}
  true
else
  echo ${BAR}
  echo " Indentation Fails"
  echo ${BAR}
  echo "${DIFF}"
  echo ${BAR}
  false
fi
