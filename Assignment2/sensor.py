from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import numpy as np

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

    print('t = {:.2f} > S3 = {:.3f} , S6 = {:.3f} , R wheel = {:.3f} , L wheel = {:.3f}'.format(t_now, obj_distance[3], obj_distance[6], wheels_velo[0], wheels_velo[1]))
    if keyboard.is_pressed('esc'):
        break

sim.stopSimulation()
print('\nProgram ended\n')