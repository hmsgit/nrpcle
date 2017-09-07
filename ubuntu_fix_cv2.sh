#!/bin/bash
# This imports cv2 to the platform_venv on ubuntu. Use this script on local installations only.
# You should have installed pyton-opencv via apt-get

mkdir $VIRTUAL_ENV/lib/python2.7/site-packages/ -p
cp /usr/lib/python2.7/dist-packages/cv2*.so $VIRTUAL_ENV/lib/python2.7/site-packages/
cp /usr/lib/python2.7/dist-packages/cv.py $VIRTUAL_ENV/lib/python2.7/site-packages/
