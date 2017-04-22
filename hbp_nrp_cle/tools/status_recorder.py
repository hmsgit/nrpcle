# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
This module contains a little tool to write the status logs into a csv file
"""

__author__ = 'GeorgHinkel'

import rospy
from std_msgs.msg import String
import json
import csv

NESTED_STRUCTURES = ["transferFunctionsElapsedTime", "detectorRefreshElapsedTime"]

dict_writer = None

with open('results.csv', 'w') as f:
    rospy.init_node("status_recorder")

    def callback(data):
        """
        Gets called if a new simulation status arrives
        """
        # pylint: disable=W0703,W0603
        try:
            json_str = str(data)[6:]
            print json_str
            json_data = json.loads(json_str)
            realtime = json_data["realTime"]
            if realtime == 0:
                print "Simulation not yet started"
                return

            global dict_writer

            if not dict_writer:
                keys = []
                for k in json_data:
                    if not k in NESTED_STRUCTURES:
                        keys.append(k)
                    else:
                        nested_struc = json_data[k]
                        keys += nested_struc.keys()

                print keys
                dict_writer = csv.DictWriter(f, keys)
                dict_writer.writeheader()

            row = dict()
            for k in json_data:
                if not k in NESTED_STRUCTURES:
                    row[k] = json_data[k]
                else:
                    nested_struc = json_data[k]
                    for elem in nested_struc:
                        row[elem] = nested_struc[elem]

            dict_writer.writerows([row])
        except Exception, e:
            print str(e)

    rospy.Subscriber("/ros_cle_simulation/status", String, callback)
    rospy.spin()
    f.close()
