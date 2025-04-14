from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import numpy as np
from math import sqrt, atan2, cos, sin

R = 97.5e-3
L = 381e-3
K1 = 1
K2 = 1
K3 = 0.5
omega_norm = 1.5

def getSensorsHandle():
    sensorsHandle = np.array([])
    for i in range(16):
        sensorHandle = sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
    return sensorsHandle

def getDistances(sensorsHandle):
    Distances = np.array([])
    for i in range(16):
        detectionState, _, detectedPoint, _, _ = sim.readProximitySensor(sensorsHandle[i])
        distanceValue = detectedPoint[2] if detectionState else 2.0
        Distances = np.append(Distances, distanceValue)
    return Distances

def getPose():
    robotHandle = sim.getObject('/PioneerP3DX')
    position = sim.getObjectPosition(robotHandle, -1)
    orientation = sim.getObjectOrientation(robotHandle, -1)
    return position, orientation[2]

def getTargetPose(diskHandle):
    position = sim.getObjectPosition(diskHandle, -1)
    orientation = sim.getObjectOrientation(diskHandle, -1)
    return position, orientation[2]

def getMotorsHandle():
    return sim.getObject('/PioneerP3DX/rightMotor'), sim.getObject('/PioneerP3DX/leftMotor')

def getDiskHandle():
    return sim.getObject('/target')

def setRobotMotion(motorsHandle, veloCmd):
    sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])

def velocityGenerator(theta, x_dot_c, y_dot_c, gamma_dot_c):
    J = np.array([
        [(R/2)*cos(theta), (R/2)*cos(theta)],
        [(R/2)*sin(theta), (R/2)*sin(theta)],
        [R/(2*L), -R/(2*L)]
    ])
    velocity_vector = np.array([x_dot_c, y_dot_c, gamma_dot_c])
    wheels_velo = np.linalg.pinv(J) @ velocity_vector
    return wheels_velo

print('Program started')
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle()
motors_handle = getMotorsHandle()
disk_handle = getDiskHandle()
wheels_velo = [0.0, 0.0]

time_start = sim.getSimulationTime()
while True:
    obj_distance = getDistances(sensors_handle)
    setRobotMotion(motors_handle, wheels_velo)

    position, theta_ref = getTargetPose(disk_handle)
    x_ref, y_ref, _ = position

    position, theta_act = getPose()
    x_act, y_act, _ = position
    
    error_x = x_ref - x_act
    error_y = y_ref - y_act
    error_theta = theta_ref - theta_act
    error_toleransi = 0.05
    print(f'Error: {error_x}, {error_y}, {error_theta}, {error_toleransi}')

    angle_to_target = atan2(error_y, error_x)
    angle_diff = atan2(sin(angle_to_target - theta_act), cos(angle_to_target - theta_act))

    theta = atan2(error_y, error_x)
    if(abs(error_x) < error_toleransi and abs(error_y) < error_toleransi):
        x_dot_center = 0
        y_dot_center = 0
        theta_dot_center = K3 * error_theta
    else:
        x_dot_center = K1 * error_x
        y_dot_center = K2 * error_y
        theta_dot_center = theta - theta_act

    while theta_dot_center > np.pi:
        theta_dot_center -= 2 * np.pi
    while theta_dot_center < -np.pi:
        theta_dot_center += 2 * np.pi

    print(f'Velocity: {x_dot_center}, {y_dot_center}, {theta_dot_center}')

    wheels_velo = velocityGenerator(theta, x_dot_center, y_dot_center, theta_dot_center)
    omega_max = max(abs(wheels_velo[0]), abs(wheels_velo[1]))
    if omega_max > omega_norm:
        wheels_velo = (omega_norm / omega_max) * wheels_velo

    if keyboard.is_pressed('esc'):
        break

sim.stopSimulation()
print('\nProgram ended\n')