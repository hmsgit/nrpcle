"""
This package contains the python code common to all.
"""


class UserCodeException(Exception):
    """
    General exception class returning a meaningful message
    to the ExD frontend when user code fails to be loaded or run.

    :param message: message that needs to be forwarded to the frontend.
    :param error_type: Type of error (like 'CLE Error')
    """
    def __init__(self, message, error_type):
        super(UserCodeException, self).__init__(message)
        self.error_type = error_type

    def __str__(self):
        return "{0} ({1})".format(repr(self.message), self.error_type)
