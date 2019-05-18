import unittest
import pylsm9ds1

class TestLsm9ds1Methods(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._init = pylsm9ds1.init(0, 0, 0, 0)

    def test_read_sub_device(self):
        device = pylsm9ds1.read_sub_device()
        self.assertEqual(device, 104)

if __name__ == '__main__':
    unittest.main()
