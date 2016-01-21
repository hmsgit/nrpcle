#!/bin/bash
# This imports cv2 to the platform_venv on ubuntu. Use this script on local installations only.
# You should have installed pyton-opencv via apt-get

mkdir platform_venv/lib/python2.7/site-packages/ -p
cp /usr/lib/python2.7/dist-packages/cv* platform_venv/lib/python2.7/site-packages/
