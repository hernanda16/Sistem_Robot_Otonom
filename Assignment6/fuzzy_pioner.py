from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import numpy as np
import matplotlib.pyplot as plt

num_inputs = 11
x_values = [i * 0.1 for i in range(num_inputs)]

def create_membership(n_set, range_min, range_max, step):
    member_values = []
    for i in range(n_set):
        member_values.append(range_min + (range_max - range_min) * i / (n_set - 1))
    return member_values

def create_fuzzy_membership(member_values, x_values):
    fuzzy_membership = []
    for i in range(len(member_values)):
        y = [0.0] * len(x_values)
        for j, x in enumerate(x_values):
            if i == 0:
                a = member_values[i]
                b = member_values[i + 1]
                if x < a:
                    y[j] = 1.0
                elif a <= x <= b:
                    y[j] = (b - x) / (b - a)
                else:
                    y[j] = 0.0
            elif i == len(member_values) - 1:
                a = member_values[i - 1]
                b = member_values[i]
                if x > b:
                    y[j] = 1.0
                elif a <= x <= b:
                    y[j] = (x - a) / (b - a)
                else:
                    y[j] = 0.0
            else:
                a = member_values[i - 1]
                b = member_values[i]
                c = member_values[i + 1]
                if a <= x <= b:
                    y[j] = (x - a) / (b - a)
                elif b < x <= c:
                    y[j] = (c - x) / (c - b)
        fuzzy_membership.append(y)
    return fuzzy_membership

def getSensorsHandle(sim):
    sensorsHandle = []
    for i in range(16):
        sensorHandle = sim.getObject('/PioneerP3DX/ultrasonicSensor[' + str(i) + ']')
        sensorsHandle.append(sensorHandle)
    _, _, _, _, _ = sim.handleProximitySensor(sim.handle_all)
    return sensorsHandle

def getDistances(sim, sensorsHandle):
    Distances = []
    for i in range(16):
        detectionState, _, detectedPoint, _, _ = sim.readProximitySensor(sensorsHandle[i])
        distanceValue = detectedPoint[2]
        if detectionState == False:
            distanceValue = 1.0
        Distances.append(distanceValue)
    return Distances

def getMotorsHandle(sim):
    motorRightHandle = sim.getObject('/PioneerP3DX/rightMotor')
    motorLeftHandle = sim.getObject('/PioneerP3DX/leftMotor')
    return motorRightHandle, motorLeftHandle

def setRobotMotion(sim, motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])

print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle(sim)
motors_handle = getMotorsHandle(sim)

wheels_velo = [0.0, 0.0]
time_start = sim.getSimulationTime()

member_values = create_membership(2, 0, 1.0, 0.1)
fuzzy_membership = create_fuzzy_membership(member_values, x_values)

plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

line_near, = ax.plot([], [], label='Near', color='blue', alpha=0.5)
line_far, = ax.plot([], [], label='Far', color='green', alpha=0.5)
colors = ['red', 'orange', 'purple', 'brown']
distance_lines = [ax.axvline(x=0, color=colors[i], linestyle='--', label=f'Sensor {i} Distance') for i in range(4)]

ax.set_title('Sensor Membership Visualization')
ax.set_xlabel('Distance')
ax.set_ylabel('Membership Degree')
ax.legend()
ax.grid(True)

while True:
    t_now = sim.getSimulationTime() - time_start
    obj_distance = getDistances(sim, sensors_handle)
    setRobotMotion(sim, motors_handle, wheels_velo)

    sensor_being_checked = [0, 2, 5, 7]

    for sensor_idx, sensor in enumerate(sensor_being_checked):
        distance = obj_distance[sensor]
        distance = max(0.0, min(distance, 1.0))

        index = int(distance / 0.1)
        index = max(0, min(index, len(x_values) - 1))

        membership_degrees = []
        for set_idx in range(len(fuzzy_membership)):
            membership_degree = fuzzy_membership[set_idx][index]
            membership_degrees.append(membership_degree)

        decision = 'Near' if membership_degrees[0] > membership_degrees[1] else 'Far'

        distance_lines[sensor_idx].set_xdata(distance)
        distance_lines[sensor_idx].set_label(f'Sensor {sensor} ({decision})')

    line_near.set_data(x_values, fuzzy_membership[0])
    line_far.set_data(x_values, fuzzy_membership[1])

    ax.relim()
    ax.autoscale_view()
    ax.legend()
    plt.pause(0.01)

sim.stopSimulation()
print('\nProgram ended\n')
