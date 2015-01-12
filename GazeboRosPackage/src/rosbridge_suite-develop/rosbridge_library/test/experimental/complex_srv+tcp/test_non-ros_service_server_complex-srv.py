#!/usr/bin/python

#pylint: skip-file

import sys
import socket
import time
from random import randint
from rosbridge_library.util import json


# variables begin ########################################
# these parameters should be changed to match the actual environment           #
#

tcp_socket_timeout = 10                          # seconds
max_msg_length = 20000                           # bytes

rosbridge_ip = "localhost"                       # hostname or ip
rosbridge_port = 9090                            # port as integer

service_type = "rosbridge_library/TestNestedService"
service_name = "nested_srv"                      # service name

send_fragment_size = 1000
# delay between sends to rosbridge is not needed anymore, if using my
# version of protocol (uses buffer to collect data from stream)
send_fragment_delay = 0.000  # 1
receive_fragment_size = 10
receive_message_intervall = 0.0

# variables end ##########################################


# service_calculation begin ##############################
# change this function to match whatever service should be provided            #
#

def calculate_service_response(request):
    request_object = json.loads(request)
                                # parse string for service request
    args = request_object["args"]

    message = {"data": {"data": 42.0}}

    """
    IMPORTANT!
    use base64 encoding to avoid JSON-parsing problems!
    --> use .decode("base64","strict") at client side
    """
    # message = message.encode('base64','strict')
    service_response_data = message

    response_object = {"op": "service_response",
                       "id": request_object["id"],
                       "data":
                       service_response_data
                       }
    response_message = json.dumps(response_object)
    return response_message

# service_calculation end ################################


# helper functions / and variables begin #################
# should not need to be changed (but could be improved )                       #
#
buffer = ""


def connect_tcp_socket():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                             # connect to rosbridge
    tcp_sock.settimeout(tcp_socket_timeout)
    tcp_sock.connect((rosbridge_ip, rosbridge_port))
    return tcp_sock


def advertise_service():
    advertise_message_object = {"op": "advertise_service",
                                "type": service_type,
                                "service": service_name,
                                "fragment_size": receive_fragment_size,
                                "message_intervall": receive_message_intervall
                                }
    advertise_message = json.dumps(advertise_message_object)
    tcp_socket.send(str(advertise_message))


def unadvertise_service():
    unadvertise_message_object = {"op": "unadvertise_service",
                                  "service": service_name
                                  }
    unadvertise_message = json.dumps(unadvertise_message_object)
    tcp_socket.send(str(unadvertise_message))


def wait_for_service_request():
    data = None
    global buffer

    try:
        done = False
        global buffer
        while not done:
            incoming = tcp_socket.recv(max_msg_length)
                                       # get data from socket
            if incoming == '':
                print "connection closed by peer"
                sys.exit(1)
            buffer = buffer + \
                incoming
            try:
                data_object = json.loads(buffer)
                if data_object["op"] == "call_service":
                    data = buffer
                    done = True
                    return data
            except Exception as e:
                # print "direct_access error:"
                # print e
                pass

            # print "trying to defragment"
            try:
                result_string = buffer.split("}{")
                                             # split buffer into fragments and
                                             # re-fill with curly brackets
                result = []
                for fragment in result_string:
                    if fragment[0] != "{":
                        fragment = "{" + fragment
                    if fragment[len(fragment) - 1] != "}":
                        fragment = fragment + "}"
                    result.append(json.loads(fragment))

                try:
                    fragment_count = len(result)
                    announced = int(result[0]["total"])

                    if fragment_count == announced:
                        reconstructed = ""
                        sorted_result = [
                            None] * fragment_count                 # sort fragments..
                        unsorted_result = []
                        for fragment in result:
                            unsorted_result.append(fragment)
                            sorted_result[int(fragment["num"])] = fragment

                        for fragment in sorted_result:
                            reconstructed = reconstructed + fragment["data"]

                        # print "reconstructed", reconstructed
                        buffer = ""
                        done = True
                        print "reconstructed message from", len(result), "fragments"
                        # print reconstructed
                        return reconstructed
                except Exception as e:
                    print "not possible to defragment:", buffer
                    print e
            except Exception as e:
                print "defrag_error:", buffer
                print e
                pass
    except Exception as e:
        # print "network-error(?):", e
        pass
    return data


def send_service_response(response):
    tcp_socket.send(response)


def list_of_fragments(full_message, fragment_size):
    message_id = randint(0, 64000)
                         # generate random message id
    fragments = []
        # generate list of data fragments
    cursor = 0
    while cursor < len(full_message):
        fragment_begin = cursor
        if len(full_message) < cursor + fragment_size:
            fragment_end = len(full_message)
            cursor = len(full_message)
        else:
            fragment_end = cursor + fragment_size
            cursor += fragment_size
        fragment = full_message[fragment_begin:fragment_end]
        fragments.append(fragment)

    fragmented_messages_list = []
        # generate list of fragmented messages (including headers)
    if len(fragments) > 1:
        for count, fragment in enumerate(fragments):
            fragmented_message_object = {"op": "fragment",
                                         "id": str(message_id),
                                         "data": str(fragment),
                                         "num": count,
                                         "total": len(fragments)
                                         }
            fragmented_message = json.dumps(fragmented_message_object)
                                            # create JSON-object from
                                            # python-object for each fragment
                                            # message
            fragmented_messages_list.append(fragmented_message)
                                            # append JSON-object to list of
                                            # fragmented messages
    else:
        fragmented_messages_list.append(str(fragment))
    return fragmented_messages_list

# helper functions end ###################################


# script begin ###########################################
# should not need to be changed (but could be improved )                       #
#

tcp_socket = connect_tcp_socket()
                                # open tcp_socket
advertise_service()
                  # advertise service in ROS (via rosbridge)
print "service provider started and waiting for requests"

try:
    while True:
        data = None
        try:
            data = wait_for_service_request()
                                            # receive request from rosbridge
            if data == '':
                break
            elif data is not None and len(data) > 0:
                response = calculate_service_response(
                    data)

                print "response calculated, now splitting into fragments.."
                fragment_list = list_of_fragments(
                    response, send_fragment_size)

                print "sending", len(fragment_list), "messages as response"
                for fragment in fragment_list:
                # print "sending:" ,fragment
                    send_service_response(fragment)
                                          # send service_response to rosbridge
                                          # (or fragments; just send any list
                                          # entry)
                    time.sleep(send_fragment_delay)
                               # (not needed if using patched rosbridge protocol.py)
        except Exception as e:
            print e
            pass
except KeyboardInterrupt:
    try:
        unadvertise_service()
                            # unadvertise service
        tcp_socket.close()
                         # close tcp_socket
    except Exception as e:
        print e
    print "non-ros_service_server stopped because user pressed \"Ctrl-C\""
