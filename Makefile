#modules that have tests
TEST_MODULES=python_cle/python_cle/

#modules that are installable (ie: ones w/ setup.py)
INSTALL_MODULES=python_cle

#packages to cover
COVER_PACKAGES=python_cle

#documentation to build
DOC_MODULES=python_cle/doc

##### DO NOT MODIFY BELOW #####################

CI_REPO?=ssh://bbprelman@bbpcode.epfl.ch/platform/ContinuousIntegration.git
CI_DIR?=ContinuousIntegration

FETCH_CI := $(shell \
		if [ ! -d $(CI_DIR) ]; then \
			git clone $(CI_REPO) $(CI_DIR) > /dev/null ;\
		fi;\
		echo $(CI_DIR) )
include $(FETCH_CI)/python/common_makefile
