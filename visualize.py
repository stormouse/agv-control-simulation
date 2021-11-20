# from mathutils import Vector
from PID import PID
import matplotlib.pyplot as plt

current_temperature = 15
desired_temperature = 30

current_heater_power = 0
max_heater_power = 85
min_heater_power = 0

kP = 0.1
kI = 0.01
kD = 0.8

def update_temperature(temperature, heater_power, delta_time):
    K = 0.3                             # thermal conductivity coefficient
    A = 1.0                             # area of conduction
    t_diff = heater_power - temperature # temperature gradient
    rate = K * A * t_diff
    return temperature + rate * delta_time

DELTA_TIME = 0.05
x = []
y1 = []
y2 = []
for t in range(1, 501):
    controller = PID(DELTA_TIME, max_heater_power, min_heater_power, kP, kD, kI)
    current_heater_power = controller.calculate(desired_temperature, current_temperature)
    current_temperature = update_temperature(current_temperature, current_heater_power, DELTA_TIME)
    print('# time: {:.2f}, env_T: {:.2f}, heater: {:.2f}, PID: {}'.format( 
            t * DELTA_TIME, 
            current_temperature, 
            current_heater_power,
            controller.describe()))
    x.append(t)
    y1.append(current_temperature)
    y2.append(current_heater_power)

plt.plot(x, y1, color='b')
plt.plot(x, y2, color='orange')
plt.plot(x, [desired_temperature for _ in x], color='grey')
plt.show()
