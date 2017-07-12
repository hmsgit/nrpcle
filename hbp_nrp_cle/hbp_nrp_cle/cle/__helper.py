# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
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
This module contains CLE helper functionality shared among SerialClosedLoopEngine and
ClosedLoopEngine implementations
"""

from hbp_nrp_cle.tf_framework import ITransferFunctionManager

__author__ = 'GeorgHinkel'


def get_tf_elapsed_times(tfm):
    """
    Gets the Transfer Functions from the given Transfer Function manager in a dictionary

    :param tfm: The Transfer Function Manager
    :return: A dictionary with the elapsed times
    """
    times = {}
    assert isinstance(tfm, ITransferFunctionManager)
    for tf in tfm.transfer_functions():
        times[tf.name] = tf.elapsed_time
    return times
