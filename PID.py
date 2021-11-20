"""
A Python Rewriting of https://gist.github.com/bradley219/5373998

Same license as Bradley's original code is applied to this snippet.

Please see the gist for license details.
"""

class PID(object):
    def __init__(self, dt, vMax, vMin, Kp, Kd, Ki):
        self._dt = dt
        self._max = vMax
        self._min = vMin
        self._kp = Kp
        self._kd = Kd
        self._ki = Ki
        self._last_error = 0
        self._integral = 0

    def calculate(self, setpoint, pv):        
        # error term
        err = setpoint - pv
        
        # proportional term
        pOut = self._kp * err

        # integral term
        self._integral += err * self._dt
        iOut = self._ki * self._integral

        # derivative term
        d = (err - self._last_error) / self._dt
        dOut = self._kd * d

        # aggregated output
        output = pOut + iOut + dOut
        if output < self._min:
            output = self._min
        elif output > self._max:
            output = self._max

        self._last_error = err

        return output

    def describe(self):
        return 'LastError={:.2f}, Integral={:.2f}'.format(self._last_error, self._integral)
