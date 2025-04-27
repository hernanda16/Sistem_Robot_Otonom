import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

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

def inverseKinematic(v, w):
    R = 97.5e-3 
    L = 381e-3
    T = np.array([[R/2, R/2],
              [R/(2*L), -R/(2*L)]])
    T_inv = np.linalg.inv(T)
    return np.dot(T_inv, np.array([v, w]))

# =============================================================

def triangleMF(x, a, b, c, FD0, FD2):
    if x < a:
        FD = FD0
    elif x >= a and x < b:
        FD = (x - a) / (b - a)
    elif x >= b and x < c:
        FD = (c - x) / (c - b)
    elif x >= c:
        FD = FD2
    return FD

def dis_MF(x):
    near = triangleMF(x, 0.2, 0.2, 0.3, 1, 0)
    far = triangleMF(x, 0.2, 0.3, 0.3, 0, 1)
    y = np.array([[near], [far]], dtype=float)
    return y
lt = ["near", "far"]

# =============================================================

fig = None
ax = None

def plot_distance_membership_functions(init=False):
    global fig, ax
    if init or fig is None:
        plt.ion()  
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_title('Distance Membership Functions')
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Membership Degree')
        ax.grid(True)
        ax.set_ylim(-0.1, 1.1)
        plt.figtext(0.5, 0.01, "Press 'q' to terminate", ha="center")
        
        def on_key(event):
            if event.key == 'q':
                print("Terminating simulation...")
                sim.stopSimulation()
                plt.close('all')
                exit()
                
        fig.canvas.mpl_connect('key_press_event', on_key)
        
    x_values = np.arange(0.0, 1, 0.01)
    near_values = [dis_MF(x)[0] for x in x_values]
    far_values = [dis_MF(x)[1] for x in x_values]
    
    ax.clear()
    
    ax.plot(x_values, near_values, 'b-', linewidth=2, label='Near')
    ax.plot(x_values, far_values, 'r-', linewidth=2, label='Far')
    ax.legend()
    ax.set_title('Distance Membership Functions')
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Membership Degree')
    ax.grid(True)
    ax.set_ylim(-0.1, 1.1)
    
    fig.canvas.draw_idle()
    plt.pause(0.001)

fig_crisp = None
ax_crisp = None
crisp_out_history = []  # Store crisp_out values over time

def plot_crisp_output(init=False, crisp_out=None):
    global fig_crisp, ax_crisp, crisp_out_history

    if init or fig_crisp is None:
        plt.ion()
        fig_crisp, ax_crisp = plt.subplots(figsize=(10, 6))
        ax_crisp.set_title('Crisp Output Over Time')
        ax_crisp.set_xlabel('Iteration')
        ax_crisp.set_ylabel('Crisp Output (PWM)')
        ax_crisp.grid(True)
        plt.figtext(0.5, 0.01, "Press 'q' to terminate", ha="center")

    if crisp_out is not None:
        crisp_out_history.append(crisp_out)  # Append new crisp_out value

    ax_crisp.clear()
    left_crisp = [co[0] for co in crisp_out_history]
    right_crisp = [co[1] for co in crisp_out_history]
    ax_crisp.plot(left_crisp, 'b-', linewidth=2, label='Left Crisp Output (PWM)')
    ax_crisp.plot(right_crisp, 'r-', linewidth=2, label='Right Crisp Output (PWM)')
    ax_crisp.legend()
    ax_crisp.grid(True)
    fig_crisp.canvas.draw_idle()
    plt.pause(0.001)

# =============================================================
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

sensors_handle = getSensorsHandle(sim)
motors_handle = getMotorsHandle(sim)

wheels_velo = [0.0, 0.0]
# =============================================================
# plot_distance_membership_functions(init=True)
# plot_crisp_output(init=True)
singleton_PWM_outputs = np.array([[-5], [5]], dtype=float)
weights = [0.5, 1.0, 1.5]
sensor_being_checked = [1, 2, 3, 4, 5, 6]
rule_table_left = np.array([[0, 0], [1, 1]], dtype=int)
rule_table_right = np.array([[0, 1], [0, 1]], dtype=int)
# =============================================================

while(True):
    output_singleton = []
    crisp_out = []

    obj_distance = getDistances(sim, sensors_handle)
    for sensor_idx, sensor in enumerate(sensor_being_checked):
        distance = obj_distance[sensor]
        distance = max(0.0, min(1.0, distance))
        output = dis_MF(distance)
        
        # print(f'{output[0][0]:.2f}, {output[1][0]:.2f}')

        output_singleton.append([output[0][0], output[1][0]])

    num_left = 0
    den_left = 0
    num_right = 0
    den_right = 0
    for idx in range(len(output_singleton[:3])):
        left_sensor = output_singleton[idx]
        right_sensor = output_singleton[len(output_singleton) - idx - 1]

        for r, rval in enumerate(right_sensor):
            for l, lval in enumerate(left_sensor):
                tab_idx_left = rule_table_left[r][l]
                tab_idx_right = rule_table_right[r][l]

                fd1andfd2 = float(min(lval, rval))

                # print(f'{idx} IF input 1 {lt[l]} FD = {lval:.2f} AND input 2 {lt[r]} FD = {rval:.2f} THEN FD of {singleton_PWM_outputs[tab_idx_left][0]} (MF idx {tab_idx_left}) = {fd1andfd2:.2f}')

                num_left = num_left + (fd1andfd2 * singleton_PWM_outputs[tab_idx_left][0])
                den_left = den_left + fd1andfd2
                num_right = num_right + (fd1andfd2 * singleton_PWM_outputs[tab_idx_right][0])
                den_right = den_right + fd1andfd2

        crisp_left = num_left / den_left if den_left > 0 else 0
        crisp_right = num_right / den_right if den_right > 0 else 0
        crisp_out.append([crisp_left, crisp_right]) 

    print(crisp_out)

    weighted_crisp = [0, 0]
    for i in range(len(crisp_out)):
        weighted_crisp[0] += crisp_out[i][0] * weights[i]
        weighted_crisp[1] += crisp_out[i][1] * weights[i]
    weighted_crisp[0] /= sum(weights)
    weighted_crisp[1] /= sum(weights)

    # print(f'Weighted crisp output: {weighted_crisp:.2f}')

    # normalize the velocity from -1 to 1
    max_velocity = 2  # Assuming the maximum absolute velocity is 5
    weighted_crisp[0] = max(-1, min(1, weighted_crisp[0] / max_velocity))
    weighted_crisp[1] = max(-1, min(1, weighted_crisp[1] / max_velocity))
    
    wheels_velo = weighted_crisp[1], weighted_crisp[0]
    setRobotMotion(sim, motors_handle, wheels_velo)

    plot_crisp_output(crisp_out=weighted_crisp)
    # plot_distance_membership_functions()