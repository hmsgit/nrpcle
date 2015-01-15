#!/usr/bin/python

#pylint: skip-file

import socket
from rosbridge_library.util import json


# variables begin ########################################
# these parameters should be changed to match the actual environment
#
client_socket_timeout = 6                       # seconds
max_msg_length = 2000000                        # bytes

rosbridge_ip = "localhost"                      # hostname or ip
rosbridge_port = 9090                           # port as integer

service_name = "nested_srv"                     # service name
# request_byte_count = 5000
receiving_fragment_size = 1000
receive_message_intervall = 0.0

# variables end ##########################################


#

def request_service():
    service_request_object = {"op": "call_service",
                              "service": "/" +
                              service_name,
                              # select service
                              "fragment_size": receiving_fragment_size,
                              # optional: tells rosbridge to send fragments if
                              # message size is bigger than requested
                              "message_intervall": receive_message_intervall,
                              "args": {"pose": {"position": {"y": 0.0, "x": 0.0, "z": 0.0},
                                                "orientation": {"y": 0.0,
                                                                "x": 0.0,
                                                                "z": 0.0,
                                                                "w": 0.0}}}
                              }
    service_request = json.dumps(service_request_object)
    print "sending JSON-message to rosbridge:", service_request
    sock.send(service_request)

#


# script begin ###########################################
# should not need to be changed (but could be improved ;) )
#
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                         # connect to rosbridge
    sock.settimeout(client_socket_timeout)
    sock.connect((rosbridge_ip, rosbridge_port))

    request_service()
                    # send service_request

    incoming = None
    buffer = ""
    done = False
    result = None
    reconstructed = None
    while not done:
        try:
            incoming = sock.recv(max_msg_length)
                                 # receive service_response from rosbridge
            if buffer == "":
                buffer = incoming
                if incoming == "":
                    print "closing socket"
                    sock.close()
                    break
            else:
                buffer = buffer + incoming
            # print "buffer-length:", len(buffer)
            try:
                data_object = json.loads(buffer)
                if data_object["op"] == "service_response":
                    reconstructed = buffer
                    done = True
            except Exception as e:
                # print "direct access to JSON failed.."
                # print e
                pass
            try:
                # print "defragmenting incoming messages"
                result_string = buffer.split("}{")
                                             # split buffer into fragments and
                                             # re-fill curly brackets
                result = []
                for fragment in result_string:
                    if fragment[0] != "{":
                        fragment = "{" + fragment
                    if fragment[len(fragment) - 1] != "}":
                        fragment = fragment + "}"
                    try:
                        result.append(json.loads(fragment))
                                      # try to parse json from string, and
                                      # append if successful
                    except Exception as e:
                        # print e
                        # print result_string
                        raise

                fragment_count = len(result)
                print "fragment_count:", fragment_count
                announced = int(result[0]["total"])
                if fragment_count == announced:
                    # sort fragments
                    sorted_result = [None] * fragment_count
                    unsorted_result = []
                    for fragment in result:
                        unsorted_result.append(fragment)
                        sorted_result[int(fragment["num"])] = fragment
                    reconstructed = ''
                    for fragment in sorted_result:
                        reconstructed = reconstructed + fragment["data"]
                    done = True
            except Exception as e:
                # print e
                pass
        except Exception as e:
#            print e
            pass

    returned_data = json.loads(reconstructed)
                               # when service response is received --> access
                               # it (as defined in srv-file)
    if returned_data["values"] is None:
        print "response was None -> service was not available"
    else:
        print "received:"
        print returned_data

except Exception as e:
    print "ERROR - could not receive service_response"
    print e

sock.close()
           # close socket
