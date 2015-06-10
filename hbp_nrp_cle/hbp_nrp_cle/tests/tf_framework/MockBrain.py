"""
This module contains a mockup for a brain model
"""

__author__ = 'GeorgHinkel'


class MockPopulation(object):

    def __init__(self, items):
        self.__items = items

    @property
    def items(self):
        return self.__items

    def __getitem__(self, item):
        return MockPopulation(self.items[item])