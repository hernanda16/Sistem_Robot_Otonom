from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import numpy as np
import matplotlib.pyplot as plt

K1 = -0.5
K2 = -0.125
d_ref = 0.5
phi_ref = 0.0
omega_norm = 1.0

# Data logging
time_log = []
d_log = []
phi_log = []

def getSensorsHandle():
    sensorsHandle = np.array([])
    for i in range(16):
        sensorHandle = sim.getObject('/PioneerP3DX/ultrasonicSensor[' + str(i) + ']')
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
    _, _, _, _, _ = sim.handleProximitySensor(sim.handle_all)
    return sensorsHandle


def getDistances(sensorsHandle):
    Distances = np.array([])
    for i in range(16):
        detectionState, _, detectedPoint, _, _ = sim.readProximitySensor(sensorsHandle[i])
        distanceValue = detectedPoint[2]
        if detectionState == False:
            distanceValue = 2.0
        Distances = np.append(Distances, distanceValue)
    return Distances

def getMotorsHandle():
    motorRightHandle = sim.getObject('/PioneerP3DX/rightMotor')
    motorLeftHandle = sim.getObject('/PioneerP3DX/leftMotor')
    return (motorRightHandle, motorLeftHandle)

def setRobotMotion(motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])
    return

def inverseKinematic(v, w):
    R = 97.5e-3
    L = 381e-3
    T = np.array([[R/2, R/2],
              [R/(2*L), -R/(2*L)]])
    T_inv = np.linalg.inv(T)
    return np.dot(T_inv, np.array([v, w]))

print('Program started')
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle()
motors_handle = getMotorsHandle()

wheels_velo = [0.0, 0.0]
time_start = sim.getSimulationTime()

while True:
    t_now = sim.getSimulationTime() - time_start
    obj_distance = getDistances(sensors_handle)
    setRobotMotion(motors_handle, wheels_velo)
    
    # Object Distance and Orientation Estimation
    d = min(obj_distance[3], obj_distance[4])
    phi = obj_distance[5] - obj_distance[2]

    # Control 
    v = K1 * (d_ref - d)
    w = K2 * (phi_ref - phi)

    # Inverse Kinematic
    wheels_velo = inverseKinematic(v, w)

    # Velocity Normalization
    omega_max = max(abs(wheels_velo[0]), abs(wheels_velo[1]))
    if omega_max > omega_norm:
        wheels_velo[0] = omega_norm / omega_max * wheels_velo[0]
        wheels_velo[1] = omega_norm / omega_max * wheels_velo[1]
    else:
        wheels_velo[0] = wheels_velo[0]
        wheels_velo[1] = wheels_velo[1]

    # Data Logging
    time_log.append(t_now)
    d_log.append(d)
    phi_log.append(phi)

    print('d = {:.3f} , phi = {:.3f}'.format(d, phi))

    if keyboard.is_pressed('esc'):
        break

sim.stopSimulation()
print('\nProgram ended\n')

# Plot results
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(time_log, d_log, label='d (min distance)', color='b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.title('Distance Over Time')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(time_log, phi_log, label='phi (orientation difference)', color='r')
plt.xlabel('Time (s)')
plt.ylabel('Orientation Difference (m)')
plt.title('Orientation Difference Over Time')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

