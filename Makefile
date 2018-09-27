#modules that have tests
TEST_MODULES=hbp_nrp_cle/hbp_nrp_cle/

#modules that are installable (ie: ones w/ setup.py)
INSTALL_MODULES=hbp_nrp_cle

#packages to cover
COVER_PACKAGES=hbp_nrp_cle

#documentation to build
DOC_MODULES=hbp_nrp_cle/doc
DOC_REPO=--doc-repo ssh://bbpcode.epfl.ch/infra/jekylltest

PYTHON_PIP_VERSION?=pip==9.0.3

##### DO NOT MODIFY BELOW #####################

ifeq ($(NRP_INSTALL_MODE),user)
	include user_makefile
else
	CI_REPO?=git@bitbucket.org:hbpneurorobotics/admin-scripts.git
	CI_DIR?=$(HBP)/admin-scripts/ContinuousIntegration
        THIS_DIR:=$(PWD)

	FETCH_CI := $(shell \
		if [ ! -d $(CI_DIR) ]; then \
			cd $(HBP) && git clone $(CI_REPO) > /dev/null && cd $(THIS_DIR);\
		fi;\
		echo $(CI_DIR) )

	include $(FETCH_CI)/python/common_makefile
endif
