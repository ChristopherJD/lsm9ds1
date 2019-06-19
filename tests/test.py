#!/usr/bin/env python
""" Python tests for the lsm9d1

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

__author__ = "Christopher Jordan-Denny"
__contact__ = "jordan.denny5@gmail.com"
__copyright__ = "Copyright 2019, Christopher Jordan-Denny"
__credits__ = ["Christopher Jordan-Denny",]
__date__ = "2019/05/18"
__deprecated__ = False
__email__ =  "jordan.denny5@gmail.com"
__license__ = "GPLv3"
__maintainer__ = "Christopher Jordan-Denny"
__status__ = "Production"
__version__ = "0.0.1"

import unittest
import pylsm9ds1

class TestLsm9ds1Methods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._init = pylsm9ds1.init(LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS)

    def test_read_sub_device(self):
        temperature = pylsm9ds1.get_temp()
        self.assertEqual(temperature, -64)

if __name__ == '__main__':
    unittest.main()
