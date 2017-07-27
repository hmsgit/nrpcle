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
Global setting of neuronal simulator. PyNNN specific devices should not depend upon a concrete
package for the neuronal simulator.
"""

__author__ = 'Sebastian Krach'

from hbp_nrp_cle.brainsim import config

import pyNN.nest as sim
from pyNN.common.control import DEFAULT_TIMESTEP, DEFAULT_MIN_DELAY

import nest

# store the pynNN.setup(...) function before patching with NRP specific behavior
pynn_setup = sim.setup


def nrp_pynn_setup(timestep=DEFAULT_TIMESTEP, min_delay=DEFAULT_MIN_DELAY, **extra_params):
    """
    Override the default PyNN setup function for NRP specific behavior, this ensures consistent
    behavior even if a brain file contains a call to pyNN.setup(...).

    See PyNN documentation for parameter information.
    """

    # if an RNG seed has not been specified, the platform is not properly initialized
    if config.rng_seed is None:
        raise Exception('RNG seed has not been set for CLE brain adapter!')

    # force Nest to use one thread - currently required for the NRP to function
    extra_params['threads'] = 1

    # force Nest to generate spikes on grid (otherwise we get a Nest crash when
    # retrieving spikes natively from Nest within the NRP)
    extra_params['spike_precision'] = 'on_grid'

    # determine the total number of threads/processes needed for RNG seeds
    rng_seed_count = nest.GetKernelStatus(['total_num_virtual_procs'])[0]

    # override the RNG seed with experiment specific parameters
    extra_params['grng_seed'] = config.rng_seed
    extra_params['rng_seeds'] = [config.rng_seed] * rng_seed_count

    # call the actual PyNN setup with our overridden parameters, return rank
    return pynn_setup(timestep, min_delay, **extra_params)

# override the setup call
sim.setup = nrp_pynn_setup

# simulator accessible for import after being patched above
simulator = sim
