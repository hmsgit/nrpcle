#!/bin/bash

export IGNORE_LINT="platform_venv|build|hbp_nrp_cle/hbp_nrp_cle/bibi_config/generated|demo/"

# This script only runs static code analysis, the tests can be run separately using run_tests.sh
make run_pep8 run_pylint
