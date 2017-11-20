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
This package contains the brain adapter implementation relying on the PyNN neuronal simulator
abstraction layer but contains implementation specifics for the SpiNNaker simulator.
"""

__author__ = "Felix Schneider"

try:
    # pylint: disable=no-name-in-module,import-error
    import pyNN.spiNNaker as spynnaker
except ImportError:
    import warnings
    warnings.warn("SpiNNaker is not installed. Please install SpiNNaker before you can use the "
                  "SpiNNaker support of the CLE")
    spynnaker = None
