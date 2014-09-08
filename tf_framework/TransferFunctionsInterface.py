__author__ = 'GeorgHinkel'


class ITransferFunctionsNode(object):

    def run_neuron_to_robot(self, t):
        """
        Runs the transfer functions from the neuronal simulator towards the robot
        :param t: The simulation time
        """
        raise Exception("Not Implemented")

    def run_robot_to_neuron(self, t):
        """
        Runs the transfer functions from the world simulation towards the neuronal simulation
        :param t:  The simulation time
        """
        raise Exception("Not Implemented")

    def initialize(self, name):
        """
        Initializes the transfer Function node with the given name
        :param name: The name for this transfer function node
        """
        raise Exception("Not Implemented")