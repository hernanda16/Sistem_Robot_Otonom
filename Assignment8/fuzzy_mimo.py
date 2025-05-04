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
    return motorLeftHandle, motorRightHandle

def setRobotMotion(sim, motorsHandle, veloCmd):
    _ = sim.setJointTargetVelocity(motorsHandle[0], veloCmd[0])
    _ = sim.setJointTargetVelocity(motorsHandle[1], veloCmd[1])

def getPose():
    robotHandle = sim.getObject('/PioneerP3DX')
    position = sim.getObjectPosition(robotHandle, -1)
    orientation = sim.getObjectOrientation(robotHandle, -1)
    return position, orientation[2]

def getTargetPose(diskHandle):
    position = sim.getObjectPosition(diskHandle, -1)
    orientation = sim.getObjectOrientation(diskHandle, -1)
    return position, orientation[2]

def getDiskHandle():
    return sim.getObject('/Disc')

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

def dis_MF_angular(x):
    left = triangleMF(x, -40, -40, 0, 1, 0)
    center = triangleMF(x, -40, 0, 40, 0, 0)
    right = triangleMF(x, 0, 40, 40, 0, 1)
    y = np.array([[left], [center], [right]], dtype=float)
    return y
lt_angular = ["left", "center", "right"]

# =============================================================

figs = []
axs_list = []
pose_history = []
sensor_readings_history = []

def initialize_plots(mode='grid'):
    global figs, axs_list, fig, axs
    
    plt.ion()
    
    if mode == 'grid':
        fig, axs = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Fuzzy Navigation Monitoring', fontsize=14)
        axs = axs  # 2D array of axes
        figs = [fig]
        axs_list = [axs[0,0], axs[0,1], axs[1,0], axs[1,1]]
        fig.canvas.mpl_connect('key_press_event', on_key)
        fig.tight_layout(pad=3.0)
        plt.figtext(0.5, 0.01, "Press 'q' to terminate", ha="center")
        plt.show(block=False)

    elif mode == 'split':
        titles = ['Distance Membership Functions',
                  'Angular Membership Functions',
                  'Robot Trajectory',
                  'Linear Error Membership']
        xlabels = ['Distance (m)', 'Angle (degrees)', 'X', 'Error']
        ylabels = ['Membership Degree', 'Membership Degree', 'Y', 'Membership Degree']

        axs_list = []
        figs.clear()
        for i in range(4):
            f, ax = plt.subplots(figsize=(6, 4))
            f.canvas.manager.set_window_title(titles[i])
            ax.set_title(titles[i])
            ax.set_xlabel(xlabels[i])
            ax.set_ylabel(ylabels[i])
            ax.grid(True)
            ax.set_ylim(-0.1, 1.1 if i != 2 else None)  # No ylim for trajectory
            axs_list.append(ax)
            figs.append(f)
            f.canvas.mpl_connect('key_press_event', on_key)
            f.tight_layout()
            plt.show(block=False)

def on_key(event):
    if event.key == 'q':
        print("Terminating simulation...")
        sim.stopSimulation()
        plt.close('all')
        exit()

def update_plots(current_sensors, current_pose):
    global sensor_readings_history, pose_history, axs_list
    
    sensor_readings_history.append(current_sensors)
    pose_history.append(current_pose)
    
    # Distance MF
    ax0 = axs_list[0]
    ax0.clear()
    x_values = np.linspace(0, 1.0, 100)
    near = [dis_MF(x)[0] for x in x_values]
    far = [dis_MF(x)[1] for x in x_values]
    ax0.plot(x_values, near, 'g-', label='Near')
    ax0.plot(x_values, far, 'r-', label='Far')
    colors = plt.cm.viridis(np.linspace(0, 1, len(current_sensors)))
    for i, dist in enumerate(current_sensors):
        ax0.axvline(x=dist, color=colors[i], linestyle='--', alpha=0.7)
    ax0.legend()
    ax0.set_title('Distance Membership Functions')
    ax0.set_ylim(-0.1, 1.1)
    ax0.grid(True)
    
    # Angular MF
    ax1 = axs_list[1]
    ax1.clear()
    x_angular = np.linspace(-180, 180, 360)
    left = [dis_MF_angular(x)[0] for x in x_angular]
    center = [dis_MF_angular(x)[1] for x in x_angular]
    right = [dis_MF_angular(x)[2] for x in x_angular]
    ax1.plot(x_angular, left, 'b-', label='Left')
    ax1.plot(x_angular, center, 'g-', label='Center')
    ax1.plot(x_angular, right, 'r-', label='Right')
    ax1.legend()
    ax1.set_title('Angular Membership Functions')
    ax1.set_ylim(-0.1, 1.1)
    ax1.grid(True)

    # Trajectory
    ax2 = axs_list[2]
    ax2.clear()
    if len(pose_history) > 1:
        x = [p[0] for p in pose_history]
        y = [p[1] for p in pose_history]
        ax2.plot(x, y, 'b-', linewidth=1.5, label='Path')
        ax2.scatter(x[0], y[0], c='g', marker='s', s=100, label='Start')
        ax2.scatter(x[-1], y[-1], c='r', marker='*', s=200, label='End')
        ax2.legend()
        ax2.set_title('Robot Trajectory')
        ax2.set_aspect('equal', adjustable='datalim')
        ax2.grid(True)

    # Error MF
    ax3 = axs_list[3]
    ax3.clear()
    x_error = np.linspace(0, 1.0, 100)
    error_near = [dis_MF(x)[0] for x in x_error]
    error_far = [dis_MF(x)[1] for x in x_error]
    ax3.plot(x_error, error_near, 'm-', label='Error Near')
    ax3.plot(x_error, error_far, 'c-', label='Error Far')
    ax3.legend()
    ax3.set_title('Linear Error Membership')
    ax3.set_ylim(-0.1, 1.1)
    ax3.grid(True)

    for f in figs:
        f.canvas.draw_idle()
    plt.pause(0.001)

# =============================================================
client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(False)
sim.startSimulation()

initialize_plots(mode='split')

sensors_handle = getSensorsHandle(sim)
motors_handle = getMotorsHandle(sim)
disk_handle = getDiskHandle()
wheels_velo = [0.0, 0.0]

weights = [0.8, 1.0, 1.5]
sensor_being_checked = [1, 2, 3, 4, 5, 6]
# =============================================================
singleton_outputs_linear = np.array([[-0.5], [0], [0.5]], dtype=float)
singleton_outputs_angular = np.array([[-0.5], [0], [0.5]], dtype=float)
rule_table_linear = np.array([[0, 1], [1, 2]], dtype=int)
rule_table_angular = np.array([[1, 0], [2, 1]], dtype=int)
# =============================================================
singleton_error_linear = np.array([[0], [0.5]], dtype=float)
singleton_error_angular = np.array([[0.5], [0], [-0.5]], dtype=float)
rule_table_error_linear = np.array([[0, 0, 0], [1, 1, 1]], dtype=int)
rule_table_error_angular = np.array([[0, 1, 2], [0, 1, 2]], dtype=int)
# =============================================================

while(True):
    is_there_near = False
    output_singleton = []
    crisp_out = []

    obj_distance = getDistances(sim, sensors_handle)
    for sensor_idx, sensor in enumerate(sensor_being_checked):
        distance = obj_distance[sensor]
        distance = max(0.0, min(1.0, distance))
        output = dis_MF(distance)

        if output[0][0] > 0.3:
            is_there_near = True
        
        # print(f'{output[0][0]:.2f}, {output[1][0]:.2f}')

        output_singleton.append([output[0][0], output[1][0]])

    num_linear = 0
    den_linear = 0
    num_angular = 0
    den_angular = 0
    for idx in range(len(output_singleton[:3])):
        left_sensor = output_singleton[idx]
        right_sensor = output_singleton[len(output_singleton) - idx - 1]

        for r, rval in enumerate(right_sensor):
            for l, lval in enumerate(left_sensor):
                tab_idx_linear = rule_table_linear[r][l]
                tab_idx_angular = rule_table_angular[r][l]

                fd1andfd2 = float(min(lval, rval))

                # print(f'{idx} IF input 1 {lt[l]} FD = {lval:.2f} AND input 2 {lt[r]} FD = {rval:.2f} THEN FD of {singleton_outputs[tab_idx_linear][0]} (MF idx {tab_idx_linear}) = {fd1andfd2:.2f}')

                num_linear = num_linear + (fd1andfd2 * singleton_outputs_linear[tab_idx_linear][0])
                den_linear = den_linear + fd1andfd2
                num_angular = num_angular + (fd1andfd2 * singleton_outputs_angular[tab_idx_angular][0])
                den_angular = den_angular + fd1andfd2

        crisp_linear = num_linear / den_linear if den_linear > 0 else 0
        crisp_angular = num_angular / den_angular if den_angular > 0 else 0
        crisp_out.append([crisp_linear, crisp_angular]) 

    # print(crisp_out)

    weighted_crisp = [crisp_out[0][0], crisp_out[0][1]]
    # for i in range(len(crisp_out)):
    #     weighted_crisp[0] += crisp_out[i][0] * weights[i]
    #     weighted_crisp[1] += crisp_out[i][1] * weights[i]
    # weighted_crisp[0] /= sum(weights)
    # weighted_crisp[1] /= sum(weights)

    # print(f'Weighted crisp output: {weighted_crisp:.2f}')
    # print(f'Weighted crisp output: {weighted_crisp[0]:.2f}, {weighted_crisp[1]:.2f}')

    # ================================================================================

    position, _ = getTargetPose(disk_handle)
    x_ref, y_ref, _ = position

    position, theta_robot = getPose()
    x_act, y_act, _ = position

    theta_target = np.arctan2(y_ref - y_act, x_ref - x_act) * 180 / np.pi
    theta_robot = theta_robot * 180 / np.pi

    current_pose = (position[0], position[1])

    while theta_target > 180:
        theta_target -= 360
    while theta_target < -180:
        theta_target += 360

    while theta_robot > 180:
        theta_robot -= 360
    while theta_robot < -180:
        theta_robot += 360

    # print(f'Orientation: {theta_robot:.2f}, Target: {theta_target:.2f}')

    error_linear = np.sqrt((x_ref - x_act) ** 2 + (y_ref - y_act) ** 2)
    error_theta = theta_target - theta_robot

    # print(f'Error linear: {error_linear:.2f}, Error angular: {error_theta:.2f}')

    output_linear = dis_MF(error_linear)
    output_angular = dis_MF_angular(error_theta)

    # print(f'Output linear: {output_linear[0][0]:.2f}, {output_linear[1][0]:.2f}')
    print(f'Output angular: {output_angular[0][0]:.2f}, {output_angular[1][0]:.2f}, {output_angular[2][0]:.2f}')

    num_error_linear = 0
    den_error_linear = 0
    num_error_angular = 0
    den_error_angular = 0

    for idx, linear in enumerate(output_linear):
        for jdx, angular in enumerate(output_angular):
            tab_idx_error_linear = rule_table_error_linear[idx][jdx]
            tab_idx_error_angular = rule_table_error_angular[idx][jdx]

            fd1andfd2 = float(min(linear[0], angular[0]))

            # print(f'IF input 1 {lt[idx]} FD = {linear[0]:.2f} AND input 2 {lt_angular[jdx]} FD = {angular[0]:.2f} THEN FD of {singleton_error_linear[tab_idx_error_linear][0]} (MF idx {tab_idx_error_linear}) = {fd1andfd2:.2f}')

            num_error_linear += (fd1andfd2 * singleton_error_linear[tab_idx_error_linear][0])
            den_error_linear += fd1andfd2
            num_error_angular += (fd1andfd2 * singleton_error_angular[tab_idx_error_angular][0])
            den_error_angular += fd1andfd2

    crisp_error_linear = num_error_linear / den_error_linear if den_error_linear > 0 else 0
    crisp_error_angular = num_error_angular / den_error_angular if den_error_angular > 0 else 0
    crisp_out.clear()
    crisp_out.append([crisp_error_linear, crisp_error_angular])
    # print(crisp_out)

    # ================================================================================

    # max_velocity = 2 
    # weighted_crisp[0] = max(-1, min(1, weighted_crisp[0] / max_velocity))
    # weighted_crisp[1] = max(-1, min(1, weighted_crisp[1] / max_velocity))

    output_velocity = []

    if(is_there_near):
        print("Obstacle Avoidance", weighted_crisp[0], weighted_crisp[1])
        output_velocity = [weighted_crisp[0] / 5, weighted_crisp[1] / 5]
    else:
        print("Target Following", crisp_out[0][0], crisp_out[0][1])
        output_velocity = [crisp_out[0][0] / 5, crisp_out[0][1] / 5]

    current_sensors = [d for i, d in enumerate(obj_distance) if i in sensor_being_checked]
    update_plots(current_sensors, current_pose)
    wheels_velo = inverseKinematic(output_velocity[0], output_velocity[1])

    # wheels_velo = 0.0, 0.0
    setRobotMotion(sim, motors_handle, wheels_velo)
