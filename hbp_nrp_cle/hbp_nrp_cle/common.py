# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
