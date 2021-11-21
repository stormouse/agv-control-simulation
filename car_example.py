from PID import PID
from processing_py import *
from bezier import CubicBezier as Bezier
import time
import math

app = App(800, 600)

DEBUG_BEZIER = False 

KP = 0.62
KI = 0.01
KD = 0.08

def guess_param(curve, location, begin=0.0, end=1.0):
    def F(t):
        p = curve.point(t)
        dx = p[0] - location[0]
        dy = p[1] - location[1]
        return dx*dx + dy*dy
    
    def dF(t):
        return (F(t + 1e-3) - F(t)) * 1e3

    def d2F(t):
        return (dF(t + 1e-3) - dF(t)) * 1e3

    # Newton's method to find local minimum
    # init
    min_t = -1
    min_dist = 1e6
    
    def initial_guess():
        origin, dest = curve.point(begin), curve.point(end)
        tx = dest[0] - origin[0]
        ty = dest[1] - origin[1]
        px = location[0] - origin[0]
        py = location[1] - origin[1]
        return (tx*px+ty*py) / (tx*tx+ty*ty)
    
    t = begin + initial_guess() / (end - begin) # need a good guess, curve is non-convex

    if t < 0 or t > 1:
        return min_t, min_dist

    for i in range(8):
        # evaluate
        y = F(t)
        if y < min_dist:
            min_dist = y
            min_t = t

        # early output
        if min_dist < 1e-3:
            break

        # update
        step = -(dF(t) / d2F(t))
        if -1e-3 < step < 1e-3:
            break
        t = t + step

    return min_t, min_dist

def host_time_ms():
    return round(time.time() * 1000)

# simulation definition
TIME_SCALE = 1.0  # 1s in real world is 0.1s in sim world
class World:
    time = 0.0
    last_update = None
    path = Bezier(
        100, 100, 
        70, 540, #app.mouseX, app.mouseY, 
        710, 60, #SCREEN_WIDTH - app.mouseX, SCREEN_HEIGHT - app.mouseY,
        700, 500)

def normalize(v2):
    sqrmag = v2[0] * v2[0] + v2[1] * v2[1]
    if sqrmag < 1e-12:
        return v2
    k = 1.0 / math.sqrt(sqrmag)
    return (v2[0] * k, v2[1] * k)

class Sim:
    pid = PID(0.02, 30, -30, Kp=KP, Kd=KD, Ki=KI)
    
    # controller definitions
    control_interval_ms = 20
    last_control_update = 0.0

    # motor definitions
    motor_power = 75 # unit/s
    motor_forward = normalize(World.path.tangent(0.01))

    # servo definitions
    MIN_SERVO_INPUT = 47.5
    MAX_SERVO_INPUT = 107.5
    SERVO_DELAY_MS = 10
    servo_input = (MIN_SERVO_INPUT + MAX_SERVO_INPUT) / 2
    servo_update_time = None

    # sensor definitions
    NUM_BELTS = 15
    HALF_BELT_WIDTH = 2 # pixel
    current_belt = 0
    desired_belt = 0
    vehicle_location = World.path.point(0.01) # also sensor location

def update_world(servo_input, motor_power, ctrl_func=None):
    if World.last_update is None:
        World.last_update = host_time_ms() - 10

    delta_time = (host_time_ms() - World.last_update) * TIME_SCALE
    World.time += delta_time
    World.last_update = World.time

    # order: motor, servo, sensor(control)
    speed_mag = Sim.motor_power * (delta_time * 0.001)
    sx, sy = Sim.motor_forward[0] * speed_mag, Sim.motor_forward[1] * speed_mag
    Sim.vehicle_location = (Sim.vehicle_location[0] + sx, Sim.vehicle_location[1] + sy)

    def direction(forward, servo_input):
        angle = servo_input - (Sim.MAX_SERVO_INPUT + Sim.MIN_SERVO_INPUT) / 2
        rad = angle * math.pi / 180.0
        cos, sin = math.cos(rad), math.sin(rad)
        # rotated forward vector
        return (cos*forward[0] - sin*forward[1], sin*forward[0] + cos*forward[1])
    
    if Sim.servo_update_time is not None and World.time - Sim.servo_update_time > Sim.SERVO_DELAY_MS:
        Sim.motor_forward = direction(Sim.motor_forward, Sim.servo_input)
        Sim.servo_update_time = None

    if ctrl_func is not None and World.time - Sim.last_control_update > Sim.control_interval_ms:
        ctrl_func()
        Sim.last_control_update = World.time


def run_sensor(sensor_location, curve, half_belt_width, num_belts):
    # find closest point on curve
    t, distance = guess_param(curve, sensor_location)
    if t > 0:
        p = curve.point(t)
        n = curve.normal(t)
        tx = sensor_location[0] - p[0]
        ty = sensor_location[1] - p[1]
        sgn = tx*n[0]+ty*n[1]
        mag = math.sqrt(tx*tx+ty*ty)
        belt_id = int((mag / half_belt_width - 1) / 2)
        if sgn < 0:
            belt_id = -belt_id
        if -num_belts / 2 <= belt_id <= num_belts / 2:
            return belt_id
    return None

def reset():
    Sim.pid = PID(0.02, 30, -30, Kp=KP, Kd=KD, Ki=KI)
    Sim.motor_forward = normalize(World.path.tangent(0.01))
    Sim.servo_input = (Sim.MIN_SERVO_INPUT + Sim.MAX_SERVO_INPUT) / 2
    Sim.servo_update_time = None
    Sim.current_belt = 0
    Sim.desired_belt = 0
    Sim.vehicle_location = World.path.point(0.01)

def controller():
    Sim.current_belt = run_sensor(Sim.vehicle_location, World.path, Sim.HALF_BELT_WIDTH, Sim.NUM_BELTS)
    if Sim.current_belt is None:
        reset()
    elif Sim.current_belt != 0:
        Sim.servo_input = Sim.pid.calculate(Sim.desired_belt, Sim.current_belt) \
                + (Sim.MIN_SERVO_INPUT + Sim.MAX_SERVO_INPUT) / 2
        print('update Sim.servo_input:', Sim.servo_input)
        Sim.servo_update_time = World.time
    
def draw_references_wrt(b, location, app):
    t, distance = guess_param(b, location)
    if t >= 0.0:
        p = b.point(t)
        n = b.normal(t) 
        p1 = b.point(t + 1e-3)
        tan = (p1[0] - p[0], p1[1] - p[1])
        sq = math.sqrt(tan[0]*tan[0] + tan[1]*tan[1])
        tan = (tan[0]/sq, tan[1]/sq)
        app.stroke(255, 0, 0)
        app.line(p[0], p[1], location[0], location[1])
        for i in range(-Sim.NUM_BELTS, Sim.NUM_BELTS + 1, 2):
            s = (p[0] + n[0] * i * Sim.HALF_BELT_WIDTH, p[1] + n[1] * i * Sim.HALF_BELT_WIDTH)
            app.line(s[0] - tan[0] * 5, s[1] - tan[1] * 5, s[0] + tan[0] * 5, s[1] + tan[1] * 5)  
        belt_number = run_sensor(location, b, Sim.HALF_BELT_WIDTH, Sim.NUM_BELTS)
        if belt_number is None:
            app.textSize(18)
            app.text('Belt: None', 10, 30)
        else:
            app.textSize(18)
            app.text('Belt:' + str(belt_number), 10, 30)
            
while True:
    
    update_world(Sim.servo_input, Sim.motor_power, controller)
    
    app.background(10, 10, 10)
    app.noFill()
    app.stroke(255, 255, 0)

    # draw path
    app.bezier(
        100, 100, 
        70, 540, #app.mouseX, app.mouseY, 
        710, 60, #SCREEN_WIDTH - app.mouseX, SCREEN_HEIGHT - app.mouseY,
        700, 500)

    # draw references
    draw_references_wrt(World.path, Sim.vehicle_location, app)

    # draw vehicle
    app.fill(0, 255, 0)
    app.stroke(255, 0, 255)
    app.circle(Sim.vehicle_location[0], Sim.vehicle_location[1], 5)
    app.noFill()

    app.stroke(255, 255, 255)
    app.textSize(18)
    app.text('forward=({:.2f}, {:.2f})'.format(Sim.motor_forward[0], Sim.motor_forward[1]), 10, 50)
    app.text('servo  ={:.2f}'.format(Sim.servo_input), 10, 70)

    if DEBUG_BEZIER:
        b = Bezier(
            100, 100,
            70, 540, #app.mouseX, app.mouseY,
            710, 60, #SCREEN_WIDTH - app.mouseX, SCREEN_HEIGHT - app.mouseY,
            700, 500)

        app.stroke(0, 255, 255)
        t = 0.0
        while t <= 1.0:
            p = b.point(t)
            n = b.normal(t)
            app.line(p[0], p[1], p[0] + n[0] * 15, p[1] + n[1] * 15)
            t += 0.05

    app.redraw()
    
