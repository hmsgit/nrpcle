"""
This package contains the brain adapter implementation relying on the PyNN neuronal simulator
abstraction layer but contains implementation specifics for the NEST simulator.
"""

__author__ = 'Sebastian Krach'

from hbp_nrp_cle.brainsim import config

import pyNN.nest as sim
from pyNN.common.control import DEFAULT_TIMESTEP, DEFAULT_MIN_DELAY

import nest
import multiprocessing
import logging

logger = logging.getLogger(__name__)

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

    # default to a single thread if not specified in the setup call
    if 'threads' not in extra_params:
        extra_params['threads'] = 1

    # ensure we leave a single core/thread for the NRP backend, oversubscribing actually hurts
    # performance and we can't guarantee number of host CPU cores for local installs
    if extra_params['threads'] >= multiprocessing.cpu_count():
        extra_params['threads'] = max(1, multiprocessing.cpu_count() - 1)
        logger.warn('Limiting number of Nest threads to %i!', extra_params['threads'])

    # force Nest to generate spikes on grid (otherwise we get a Nest crash when
    # retrieving spikes natively from Nest within the NRP)
    extra_params['spike_precision'] = 'on_grid'

    # reset the Nest kernel threads/processes/RNG seeds before interacting with it
    nest.ResetKernel()

    # determine the total number of threads/processes needed for RNG seeds
    rng_seed_count = nest.GetKernelStatus(['total_num_virtual_procs'])[0] * extra_params['threads']

    # override the RNG seed with experiment specific parameters
    extra_params['grng_seed'] = config.rng_seed
    extra_params['rng_seeds'] = [config.rng_seed] * rng_seed_count

    # call the actual PyNN setup with our overridden parameters, return rank
    return pynn_setup(timestep, min_delay, **extra_params)

# override the setup call
sim.setup = nrp_pynn_setup
