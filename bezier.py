import math
# from processing_py import *

class CubicBezier(object):
    EPSILON = 1e-9

    def __init__(self, x0, y0, x1, y1, x2, y2, x3, y3):
        self._x0 = x0
        self._y0 = y0
        self._x1 = x1
        self._y1 = y1
        self._x2 = x2
        self._y2 = y2
        self._x3 = x3
        self._y3 = y3

    def point(self, t):
        u = 1 - t
        c0 = u*u*u
        c1 = 3*u*u*t
        c2 = 3*u*t*t
        c3 = t*t*t
        x = c0 * self._x0 + c1 * self._x1 + c2 * self._x2 + c3 * self._x3
        y = c0 * self._y0 + c1 * self._y1 + c2 * self._y2 + c3 * self._y3
        return (x, y)

    def tangent(self, t):
        dx = self._cubic_d(t, self._x0, self._x1, self._x2, self._x3)
        dy = self._cubic_d(t, self._y0, self._y1, self._y2, self._y3)
        return (dx, dy)

    def normal(self, t):
        dx, dy = self.tangent(t)
        return self._normalize(-dy, dx)

    def _normalize(self, x, y):
        norm = x*x + y*y
        if norm < 1e-10:
            return (0, 0)
        m = 1.0 / math.sqrt(norm)
        return (x * m, y * m)

    def _cubic_d(self, t, a, b, c, d):
        c0 = (d - 3*c + 3*b - a)
        c1 = (2*c - 4*b + 2*a)
        c2 = (b - a)
        return 3 * (c0*t*t + c1*t + c2)

#if __name__ == '__main__':
#    app = App(640, 480)
#    while True:
#        app.background(0, 0, 0)
#        app.noFill()
#        
#        app.stroke(255, 255, 0)
#        app.bezier(100, 100, app.mouseX, app.mouseY, 640-app.mouseX, 480-app.mouseY, 540, 380)
#
#        app.stroke(0, 255, 255)
#        b = CubicBezier(100, 100, app.mouseX, app.mouseY, 640-app.mouseX, 480-app.mouseY, 540, 380)
#        t = 0.0
#        while t <= 1.0:
#            p = b.point(t)
#            n = b.normal(t)
#            app.line(p[0], p[1], p[0] + n[0] * 15, p[1] + n[1] * 15)
#            t += 0.05
#        app.redraw()

