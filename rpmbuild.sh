#!/bin/sh -x

# This file is not used anymore in production (as of 2015-02-11)
# but could still be used to build RPM packages.

if [ "$(id -u)" == "0" ]; then
  echo "Build shouldn't be done as root" 1>&2
  exit 1
fi

# Retrieving package name
name=`grep Name neurorobotics.CLE.spec | awk '{print $2}'`
version=`grep Version neurorobotics.CLE.spec | awk '{print $2}'`
package=${name}-${version} 

rm -rf ./rpmbuild/ ./packages/ ${package}

mkdir ${package}
cp -r GazeboRosPackage/src ${package} 
tar -cvjf ${package}.tar.bz2 ${package} 

mkdir -p ./rpmbuild/{BUILD,RPMS,SOURCES,SPECS,SRPMS} 

mv ${package}.tar.bz2 ./rpmbuild/SOURCES/
cp neurorobotics.CLE.spec ./rpmbuild/SPECS/
rm -rf ${package}

mkdir -p ./packages/src/
mock -r epel-6-x86_64 --buildsrpm --spec ./rpmbuild/SPECS/neurorobotics.CLE.spec --sources ./rpmbuild/SOURCES/ --resultdir=./packages/src/ --verbose
mock -r epel-6-x86_64 --rebuild --cleanup-after --resultdir=./packages/"%(dist)s"/"%(target_arch)s"/ ./packages/src/neurorobotics.CLE*.el6.src.rpm --verbose
