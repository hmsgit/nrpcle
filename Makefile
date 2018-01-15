#modules that have tests
TEST_MODULES=hbp_nrp_cle/hbp_nrp_cle/

#modules that are installable (ie: ones w/ setup.py)
INSTALL_MODULES=hbp_nrp_cle

#packages to cover
COVER_PACKAGES=hbp_nrp_cle

#documentation to build
DOC_MODULES=hbp_nrp_cle/doc
DOC_REPO=--doc-repo ssh://bbpcode.epfl.ch/infra/jekylltest

##### DO NOT MODIFY BELOW #####################

ifeq ($(NRP_INSTALL_MODE),user)
	include user_makefile
else
	CI_REPO?=ssh://bbpcode.epfl.ch/platform/ContinuousIntegration.git
	CI_DIR?=ContinuousIntegration

	FETCH_CI := $(shell \
		if [ ! -d $(CI_DIR) ]; then \
			git clone $(CI_REPO) $(CI_DIR) > /dev/null ;\
		fi;\
		echo $(CI_DIR) )

	include $(FETCH_CI)/python/common_makefile
endif
