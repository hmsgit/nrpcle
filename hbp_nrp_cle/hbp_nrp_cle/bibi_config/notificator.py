"""
This module implements an utility class for managing notifications during a simulation.
"""

__author__ = 'Alessandro Ambrosano'


class Notificator(object):
    """
    Notification manager for notification created during a simulation.
    """

    # http://stackoverflow.com/questions/6287459/python-function-of-a-default-argument
    __notification_function = staticmethod(lambda y, z: ())

    @staticmethod
    def register_notification_function(notification_function):
        """
        Registers a notification function where to forward notifications.
        :param notification_function: The function to be registered.
        """
        # May also become a list of notification function if needed
        Notificator.__notification_function = staticmethod(notification_function)

    @staticmethod
    def notify(message, update_progress):
        """
        Forwards a notification to the currently registered notification function.
        :param message: Title of the first subtask. Could be empty (example: loading Virtual Room).
        :param update_progress: Boolean. Index of current subtask should be updated? (usually yes).
        """
        Notificator.__notification_function(message, update_progress)
