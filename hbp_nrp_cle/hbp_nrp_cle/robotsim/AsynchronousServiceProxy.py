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
This module contains a specialized rospy ServiceProxy that allows to call a service
asynchronously. It uses a separate thread to retrieve the result after the service call has been
sent in a blocking manner.
"""

__author__ = 'Sebastian Krach'

from rospy import ServiceProxy, is_shutdown, ROSInterruptException, ServiceException, \
    TransportException
from rospy.msg import args_kwds_to_message
from threading import Event, Thread
from concurrent.futures import Future
from sys import exc_info
import logging

logger = logging.getLogger(__name__)


class AsynchonousRospyServiceProxy(object):
    """
    An specialization of the rospy.ServiceProxy class which allows to call ros services
    asynchronously. While the invokation of the service happens in a blocking manner, the call
    does not wait for the service to terminate but passes the results to a callback method.
    """

    def __init__(self, name, service_class, persistent=True, headers=None):
        """
        Creates a new asynchronous service proxy.

        @param name: name of service to call
        @param service_class: auto-generated service class
        @param persistent: (optional) if True, proxy maintains a persistent
        connection to service. For the asynchronous retrieval of the result the connection has to
        be persistent.
        @param headers: (optional) arbitrary headers
        """
        self.__proxy = ServiceProxy(name, service_class, persistent, headers)

        self.__await_finish_thread = Thread(target=self.__retrieve_results_async)
        self.__await_finish_thread.daemon = True

        self.__retrieve_result = Event()
        self.__call_future = Future()
        self.__shutdown_result_thread = False

    def __call__(self, *args, **kwargs):
        """
        Callable-style version of the service API compatible to ROS Service Proxy. This accepts
        either a request message instance, or you can call directly with arguments to create a
        new request instance. e.g.::

          add_two_ints(AddTwoIntsRequest(1, 2))
          add_two_ints(1, 2)
          add_two_ints(a=1, b=2)

        @param args: arguments to remote service
        @param kwargs: message keyword arguments
        @raise ROSSerializationException: If unable to serialize
        message. This is usually a type error with one of the fields.
        """
        return self.call(*args, **kwargs)

    def call(self, *args, **kwds):
        """
        Call the service. This accepts either a request message instance,
        or you can call directly with arguments to create a new request instance. e.g.::

          add_two_ints(AddTwoIntsRequest(1, 2))
          add_two_ints(1, 2)
          add_two_ints(a=1, b=2)

        The first service invokation is executed in a blocking manner until the transport
        connection is established. If the connection is set to be persistent (default) subsequent
        service calls are executed asynchronously. In either case the result is passed to the
        specified callback method.

        @raise TypeError: if request is not of the valid type (Message)
        @raise ServiceException: if communication with remote service fails
        @raise ROSInterruptException: if node shutdown (e.g. ctrl-C) interrupts service call
        @raise ROSSerializationException: If unable to serialize
        message. This is usually a type error with one of the fields.
        """

        if self.__retrieve_result.is_set():
            raise ServiceException("Previous call did not return yet")

        self.__call_future = Future()
        self.__call_future.set_running_or_notify_cancel()

        # initialize transport
        if self.__proxy.transport is None:
            self.__call_future.set_result(self.__proxy.call(*args, **kwds))

        else:
            if not self.__await_finish_thread.is_alive():
                self.__await_finish_thread.start()

            # convert args/kwds to request message class
            request = args_kwds_to_message(self.__proxy.request_class, args, kwds)

            # send the actual request message
            self.__proxy.seq += 1
            self.__proxy.transport.send_message(request, self.__proxy.seq)

            self.__retrieve_result.set()

        return self.__call_future

    def close(self):
        """
        Closes the connection to the rospy service
        """

        self.__shutdown_result_thread = True
        self.__retrieve_result.set()

        self.__proxy.close()

    def __retrieve_results_async(self):
        """
        The method running in a separate thread which is responsible for retrieving the result of
        the service call and invoking the callback methods.
        """

        while not self.__shutdown_result_thread:
            try:
                self.__retrieve_result.wait()

                if self.__shutdown_result_thread:
                    return

                ## The code is copied and adapted from rospy.ServiceProxy from here
                try:
                    responses = self.__proxy.transport.receive_once()
                    if len(responses) == 0:
                        raise ServiceException(
                            "service [%s] returned no response" % self.__proxy.resolved_name)
                    elif len(responses) > 1:
                        raise ServiceException("service [%s] returned multiple responses: %s" % (
                            self.__proxy.resolved_name, len(responses)))
                except TransportException as e:
                    # convert lower-level exception to exposed type
                    if is_shutdown():
                        raise ROSInterruptException("node shutdown interrupted service call")
                    else:
                        raise ServiceException(
                            "transport error completing service call: %s" % (str(e)))
                ## The code is copied and adapted from rospy.ServiceProxy until here

                self.__retrieve_result.clear()
                self.__call_future.set_result(*responses)

            # pylint: disable=broad-except
            except Exception:
                logger.error("Error retrieving service result")
                exept_info = exc_info()[1:3]
                old_future = self.__call_future
                self.__retrieve_result.clear()
                old_future.set_exception_info(*exept_info)
