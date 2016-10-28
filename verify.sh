#!/bin/bash
# This script is designed for local usage.

# Copy open-cv binary
./ubuntu_fix_cv2.sh

export IGNORE_LINT="platform_venv|build|hbp_nrp_cle/hbp_nrp_cle/bibi_config/generated|demo/|hbp_nrp_cle/.eggs"

# This script only runs static code analysis, the tests can be run separately using run_tests.sh
make run_pep8 run_pylint
RET=$?

if [ $RET == 0 ]; then
    echo -e "\033[32mVerify sucessfull.\e[0m Run ./run_tests.sh to run the tests."
else
    echo -e "\033[31mVerify failed.\e[0m"
fi

exit $RET
