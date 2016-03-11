"""
Interface to implement when a cleanup on a parameter is needed when updating/removing a transfer
function
"""


class ICleanableTransferFunctionParameter(object):  # pragma: no cover
    """
    Implement this when a cleanup on a parameter is needed when updating/removing a
    transfer function
    """

    def cleanup(self):
        """
        Perform any cleanup operations needed.
        """
        raise NotImplementedError("This method was not implemented in the concrete implementation")
