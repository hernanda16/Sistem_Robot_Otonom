from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard
import numpy as np

wheels_velo = [0.0, 0.0]
last_key = ""
target_linear = 0.0
target_angular = 0.0
current_linear = 0.0
current_angular = 0.0

CONST_TIME_HYSTERISIS_LINEAR = 5
CONST_TIME_HYSTERISIS_ANGULAR = 1
CONST_MAX_LINEAR = 1
CONST_MAX_ANGULAR = 0.3
CONST_ACCEL_LINEAR = 0.5
CONST_ACCEL_ANGULAR = 0.5
CONST_DT = 0.1

def getMotorsHandle():
    motorRightHandle = sim.getObject("/Robot_Pemain/rightMotor")
    motorLeftHandle = sim.getObject("/Robot_Pemain/leftMotor")
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

def velocityKey(key):
    if(key == "w"):
        return (CONST_MAX_LINEAR, 0.0)
    elif(key == "s"):
        return (-CONST_MAX_LINEAR, 0.0)
    elif(key == "a"):
        return (0.0, CONST_MAX_ANGULAR)
    elif(key == "d"):
        return (0.0, -CONST_MAX_ANGULAR)
    elif(key == " "):
        return (0.0, 0.0)
    else:
        return (0.0, 0.0)
    

print('Program started')
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

motors_handle = getMotorsHandle()
time_start = sim.getSimulationTime()
time_hysterisis_start = sim.getSimulationTime()

while True:
    if(keyboard.is_pressed('esc')):
        break

    if keyboard.is_pressed("w") and last_key != "w":
        time_hysterisis_start = sim.getSimulationTime()
        last_key = "w"
    elif keyboard.is_pressed("s") and last_key != "s":
        time_hysterisis_start = sim.getSimulationTime()
        last_key = "s"
    elif keyboard.is_pressed("a") and last_key != "a":
        time_hysterisis_start = sim.getSimulationTime()
        last_key = "a"
    elif keyboard.is_pressed("d") and last_key != "d":
        time_hysterisis_start = sim.getSimulationTime()
        last_key = "d"
    elif keyboard.is_pressed(" ") and last_key != " ":
        time_hysterisis_start = sim.getSimulationTime()
        last_key = " "

    target_linear, target_angular = velocityKey(last_key)

    if target_linear > current_linear:
        current_linear = min(current_linear + CONST_ACCEL_LINEAR * CONST_DT, target_linear)
    elif target_linear < current_linear:
        current_linear = max(current_linear - CONST_ACCEL_LINEAR * CONST_DT, target_linear)

    if target_angular > current_angular:
        current_angular = min(current_angular + CONST_ACCEL_ANGULAR * CONST_DT, target_angular)
    elif target_angular < current_angular:
        current_angular = max(current_angular - CONST_ACCEL_ANGULAR * CONST_DT, target_angular)


    if(current_linear > CONST_MAX_LINEAR):
        current_linear = CONST_MAX_LINEAR
    elif(current_linear < -CONST_MAX_LINEAR):
        current_linear = -CONST_MAX_LINEAR

    if(current_angular > CONST_MAX_ANGULAR):
        current_angular = CONST_MAX_ANGULAR
    elif(current_angular < -CONST_MAX_ANGULAR):
        current_angular = -CONST_MAX_ANGULAR
    
    wheels_velo = inverseKinematic(current_linear, current_angular)
    setRobotMotion(motors_handle, wheels_velo)

    print('linear = {:.3f} , angular = {:.3f} , R wheel = {:.3f} , L wheel = {:.3f}'.format(current_linear, current_angular, wheels_velo[0], wheels_velo[1]))

    delay = CONST_DT 
    sim.wait(delay)

sim.stopSimulation()
print('\nProgram ended\n')