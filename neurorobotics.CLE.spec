%define cle_prefix opt/bbp
%define dest_dir   %{cle_prefix}/ros/groovy

Name:		neurorobotics.CLE
Version:	0.0.1
Release:	2%{?dist}
Summary:	Neurorobotics collection of ROS-Gazebo plugins

License:	Not yet defined
URL:		https://bbpteam.epfl.ch/project/spaces/display/HSP10/CLE
Source:		%{name}-%{version}.tar.bz2
Prefix:         %{cle_prefix}

BuildRequires:  environment-modules
BuildRequires:	tinyxml-devel
BuildRequires:	urdfdom-headers-devel
BuildRequires:  cmake
BuildRequires:  python-pip
BuildRequires:  python27-python
BuildRequires:  python27-python-virtualenv
BuildRequires:  protobuf-devel
BuildRequires:  log4cxx-devel
BuildRequires:  freeimage-devel
Requires: tinyxml
Requires: urdfdom
Requires: protobuf
Requires: log4cxx
Requires: freeimage

%description
The neurorobotics CLE is a collection of Gazebo/ROS plugins that aims
to let people run neural simulations within ROS

%prep
%setup

%build
source /opt/rh/python27/enable

export MODULEPATH=$MODULEPATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/modulefiles
module load boost/1.55zlib-rhel6-x86_64-gcc4.4
module load ros/hydro-rhel6-x86_64-gcc4.4
module load gazebo/4.0-rhel6-x86_64-gcc4.8.2
module load opencv/2.4.9-rhel6-x86_64-gcc4.8.2
module load sdf/2.0-rhel6-x86_64-gcc4.4
module load tbb/4.0.5-rhel6-x86_64-gcc4.4

virtualenv build_venv
. build_venv/bin/activate

pip install -i http://bbpsrv19.epfl.ch:9090/simple catkin_pkg
pip install -i http://bbpsrv19.epfl.ch:9090/simple empy
pip install -i http://bbpsrv19.epfl.ch:9090/simple PyYAML
pip install -i http://bbpsrv19.epfl.ch:9090/simple rospkg
pip install -i http://bbpsrv19.epfl.ch:9090/simple netifaces

source $ROS_SETUP_FILE

cd src
catkin_init_workspace
cd ..

catkin_make clean -DBoost_INCLUDE_DIR=$BOOST_INCLUDEDIR -DBoost_LIBRARY_DIRS=$BOOST_LIBDIR -DBoost_NO_BOOST_CMAKE=true -DTBB_INCLUDE_DIR=$TBB_INCLUDE_DIR

BOOST_ROOT=/nfs4/bbp.epfl.ch/sw/neurorobotics/boost/1.55-zlib/rhel-6.5-x86_64/gcc-4.4.7/x86_64/ catkin_make -DBoost_INCLUDE_DIR=$BOOST_INCLUDEDIR -DBoost_LIBRARY_DIRS=$BOOST_LIBDIR -DBoost_NO_BOOST_CMAKE=true -DTBB_INCLUDE_DIR=$TBB_INCLUDE_DIR

catkin_make -DCMAKE_INSTALL_PREFIX=%{dest_dir} install

%install

%{__mkdir} -p %{buildroot}/%{dest_dir}
cp -R %{dest_dir}/* %{buildroot}/%{dest_dir}

%check

%post -p /sbin/ldconfig

%postun -p /sbin/ldconfig

%files
%defattr(-,root,root,-)
/%{dest_dir}

%changelog

* Thu Nov 25 2013 Daniel Peppicelli <daniel.peppicelli@gmail.com> - 0.0.1
- Initial package

