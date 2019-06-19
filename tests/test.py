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
