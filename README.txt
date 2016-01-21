This is the implementation and architecture documentation of the Closed Loop Engine (CLE) 


make devinstall
===============
Problem: On a local system outside of the EPFL, you get the error: "Warning: no valid set_module_path.sh, do you have access to /nfs4 or /gpfs?"

Solution: do a local install of numpy, scipy, lxml and h5py and some other dependencies
$ sudo apt-get install python-dev python-h5py python-lxml autogen automake libtool build-essential autoconf libltdl7-dev libreadline6-dev libncurses5-dev libgsl0-dev python-all-dev python-numpy python-scipy python-matplotlib ipython python-pynn libxslt1-dev zlib1g-dev python-opencv

Problem:  The repository located at bbpgb019.epfl.ch is not a trusted or secure host and is being ignored. If this repository is available via HTTPS it is recommended to use HTTPS instead, otherwise you may silence this warning and allow it anyways with '--trusted-host bbpgb019.epfl.ch'.

Solution:
Newer versions of pip don't warn but throw an error when acessing a repository without https. Create a file ~/.pip/pip.conf with the content between the seperators.

Filename: ~/.pip/pip.conf
---------------------------------------
[install]
trusted-host=bbpgb019.epfl.ch
---------------------------------------
