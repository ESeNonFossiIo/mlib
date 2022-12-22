#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

conda activate ${SCRIPT_DIR}/../build/mlibpy_env
pytest ${SCRIPT_DIR}/../python/tests
conda deactivate
