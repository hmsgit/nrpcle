"""
Helper functions for CLE
"""

__author__ = 'Bernd Eckstein'

import netifaces


def get_local_ip():
    """
    Get local IP-Address

    Returns:
        Local IP of the first connected network interface or
        127.0.0.1 if no network interface is connected.
    """

    interfaces = ["eth0", "eth1", "eth2", "wlan0", "wlan1", "wifi0", "ath0", "ath1", "ppp0"]
    ip = "127.0.0.1"
    for ifname in interfaces:
        try:
            ip = netifaces.ifaddresses(ifname)[netifaces.AF_INET][0]['addr']
            break
        except IOError:
            pass
        except ValueError:
            pass
        except KeyError:
            pass
    return ip
