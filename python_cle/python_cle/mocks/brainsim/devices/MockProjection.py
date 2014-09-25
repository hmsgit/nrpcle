'''
Implementation of MockProjection
moduleauthor: Michael.Weber@fzi.de
'''

__author__ = 'MichaelWeber'


class MockProjection(object):
    """
    Represents a mocked projection
    """
    #pylint: disable=R0913
    def __init__(self, presynaptic_population, postsynaptic_population, method, source=None,
                 target=None, synapse_dynamics=None, label=None, rng=None):
        self.presynaptic_population = presynaptic_population
        self.postsynaptic_population = postsynaptic_population
        self.connector = method
        self.source = source
        self.target = target
        self.synapse_dynamics = synapse_dynamics
        self.label = label
        self.rng = rng
