import unittest

from PID import PID

class PIDTestMethods(unittest.TestCase):

    def test_convergeWhenWithoutDisturbance(self):
        _pid = PID(0.02, 107.5, 47.5, 0.12, 0.23, 0.05)
        _output = _pid.calculate(42, 0)
        self.assertTrue(_output > 0)        


if __name__ == '__main__':
    unittest.main()
