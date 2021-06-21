import setup_path 
import airsim
import numpy as np
import os
import pprint
import setup_path 
import tempfile
import math 
import csv
from datetime import datetime
import time
# Use below in settings.json with Blocks environment
def sat(theta, m, M):
    if theta < m:
        return m
    if theta > M:
        return M
    else:
        return theta
def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])
def rot(theta):
    return np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])
def alpha(t,t0):
    k = t-t0
    p =0.51
    e = 1
    c=1.6
    if k > 1:
      return c*1/k**p
    else:
      return c*(-p-1)/(e**(p+2))*k**2 +c*(2+p)*k/(e**(p+1))
"""
{
	"SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	"ClockSpeed": 1,
	
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
		  "X": 4, "Y": 0, "Z": -2
		},
		"Drone2": {
		  "VehicleType": "SimpleFlight",
		  "X": 8, "Y": 0, "Z": -2
		}

    }
}
"""
#Update data to CSV file

T = 0
Dx1 = 0
Dy1 = 0

fieldnames = ["Time", "Drone1X", "Drone1Y", "Drone2X", "Drone2Y", "Drone3X", "Drone3Y","Drone1VX", "Drone1VY", "Drone2VX", "Drone2VY", "Drone3VX", "Drone3VY"]
now = datetime.now()
#File ='D:/Unreal Projects/FormationControl/Results/'+ str(now.strftime("%X"))  + '.csv' 
File ='D:/Unreal Projects/FormationControl/Results/'+ 'data'  + '.csv' 

with open(File, 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
    
pi = 3.141592653589793
numUAV = 3
pos0 = np.array([[0 ,0 ,0],[-5, 0, 0], [5, 0 , 0]])
#w0 = np.array([0],[0],[0])
R = np.array([[0 ,-3 ,0],[-3, 0, 0], [3, 0 ,0]])
V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
time.sleep(8)
#R = np.zeros((numUAV,3))
#for i in range(numUAV):
#    pos0[i,0] = -5*(i-1)
#    R[i,0] = 5*math.cos(2*i*pi/3) 
#    pos0[i,1] = 0
#    R[i,1] = 5*math.sin(2*i*pi/3)
#    pos0[i,2] = 0
#    R[i,2] = 0

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "UAV1")
client.armDisarm(True, "UAV1")

client.enableApiControl(True, "UAV2")
client.armDisarm(True, "UAV2")

client.enableApiControl(True, "UAV3")
client.armDisarm(True, "UAV3")
#Take off
airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name="UAV1")
f1.join()

f2 = client.takeoffAsync(vehicle_name="UAV2")
f2.join()

f3 = client.takeoffAsync(vehicle_name="UAV3")
f3.join()

#state1 = client.getMultirotorState(vehicle_name="UAV1")
#s = pprint.pformat(state1)
#print("state: %s" % s)
#state2 = client.getMultirotorState(vehicle_name="Drone2")
#s = pprint.pformat(state2)
#print("state: %s" % s)

#airsim.wait_key('Press any key to move vehicles')
#f1 = client.moveToPositionAsync(0, 0, 0, -5, vehicle_name="UAV1")
#f1.join()
#f2 = client.moveToPositionAsync(0, 0, 0, -5, vehicle_name="UAV2")
#f2.join()
#f3 = client.moveToPositionAsync(0, 0, 0, -5, vehicle_name="UAV3")
#f3.join()
#f2.join()

#airsim.wait_key('Press any key to take images')
## get camera images from the car
#responses1 = client.simGetImages([
#    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
#    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")  #scene vision image in uncompressed RGB array
#print('Drone1: Retrieved images: %d' % len(responses1))
#responses2 = client.simGetImages([
#    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
#    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Drone2")  #scene vision image in uncompressed RGB array
#print('Drone2: Retrieved images: %d' % len(responses2))
#
#tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
#print ("Saving images to %s" % tmp_dir)
#try:
#    os.makedirs(tmp_dir)
#except OSError:
#    if not os.path.isdir(tmp_dir):
#        raise
#
#for idx, response in enumerate(responses1 + responses2):
#
#    filename = os.path.join(tmp_dir, str(idx))
#
#    if response.pixels_as_float:
#        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
#        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
#    elif response.compress: #png format
#        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
#    else: #uncompressed array
#        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
#        img_rgb = img1d.reshape(response.height, response.width, 3) #reshape array to 3 channel image array H X W X 3
#        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png
itr = 0
dt = 0.005
t = 0
#Target Position
rt = np.array([0, 0, -5])
vt = np.array([0, 12, 0])
omega = pi/6
S = 1 #sequence
t_0 = 0
#vt = np.array([0, 5, 0])
while t < 75: 
    t = t + dt
    itr = itr + 1
#    if rt[1] > 35:
#        R = np.array([[0 ,0 ,0],[0, -3, 0], [0, -6 ,0]])
#    if rt[1] > 90:
#        R = np.array([[0 ,0 ,0],[-3, 0, 0], [3, 0 ,0]])
#    if rt[1] > 100:
#        R = np.array([[5*math.cos(omega*t) ,5*math.sin(omega*t) ,0],[5*math.cos(omega*t + 2*pi/3) ,5*math.sin(omega*t + 2*pi/3) ,0], [5*math.cos(omega*t + 4*pi/3) ,5*math.sin(omega*t + 4*pi/3) ,0]])
#        V = np.array([[-5*omega*math.sin(omega*t) ,5*omega*math.cos(omega*t),0],[-5*omega*math.sin(omega*t+ 2*pi/3) ,5*omega*math.cos(omega*t+ 2*pi/3) ,0], [-5*omega*math.sin(omega*t+ 4*pi/3) ,5*omega*math.cos(omega*t+ 4*pi/3) ,0]])
#        vt = np.array([0, 0, 0])
#        rt = np.array([-3, 120, 0])

#    print("Time = ", t)
    if S == 1:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([0, 10, 0])
        rt = dt*vt + r
        R = np.array([[0 ,3 ,0],[-3, 0, 0], [3, 0 ,0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 , 0]])
        if rt[1] > 45:
            S = 2
            t_0 = t
    if S == 2:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([0, 10, 0])
        rt = dt*vt + r
        R = np.array([[0 ,3 ,0],[0, 0, 0], [0, -3 ,0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
        if r3.y_val>120:
            S = 3
            t_0 = t
    if S == 3:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([-10, 0, 0])
        rt = dt*vt + r
        R = np.array([[3 ,0 ,0],[0, 0, 0], [-3, 0, 0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
        if r1.x_val<-10:
            S = 4
            t_0 = t
    if S == 4:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([-10, 0, 0])
        rt = dt*vt + r
        R = np.array([[2 , 0,0],[0, -2, 0], [0, 2, 0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
        if r1.x_val<-65:
            S = 5
            t_0 = t
    if S == 5:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([-10, 0, 0])
        rt = dt*vt + r
        R = np.array([[3 , 0,0],[0, 0, 0], [-3, 0, 0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
        if r3.x_val<-75:
            S = 6
            t_0 = t
    if S == 6:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([0, -10, 0])
        rt = dt*vt + r
        R = np.array([[0 , 0,0],[0, 2, 0], [0, 4, 0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
        if r1.y_val<110:
            S = 7
            t_0 = t
    if S == 7:
        r = rt
        vt = ((alpha(t,t_0))**1.5)*np.array([0, -10, 0])
        rt = dt*vt + r
        R = np.array([[0 ,-2,0],[2, 0, 0], [-2, 0, 0]])
        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
#        if r1.y_val<110:
#            S = 7
#            t_0 = t
#    if S == 3:
#        r = rt
#        vt = alpha(t,t_0)*np.array([0, 10, 0])
#        rt = dt*vt + r
#        R = np.array([[0 ,0 ,0],[3, 0, 0], [-3, 0 ,0]])
#        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
#        if rt[1]>110:
#            S = 4
#            t_0 = t
#    if S == 4:
#        r = rt
#        vt = alpha(t,t_0)*np.array([0, 10, 0])
#        rt = dt*vt + r
#        R = np.array([[0 ,0 ,0],[0, -1.5, 0], [0, -3 ,0]])
#        V = np.array([[0 ,0 ,0],[0, 0, 0], [0, 0 ,0]])
#        if rt[1]>120:
#            S = 5
#            t_0 = t
#    print(rt)
    # Get x-y-z coordinates
    r1 = client.getMultirotorState(vehicle_name="UAV1").kinematics_estimated.position
    w1 = client.getMultirotorState(vehicle_name="UAV1").kinematics_estimated.orientation
    a1 = quaternion_to_euler(w1.w_val,w1.x_val,w1.y_val,w1.z_val)
    r2 = client.getMultirotorState(vehicle_name="UAV2").kinematics_estimated.position
    w2 = client.getMultirotorState(vehicle_name="UAV2").kinematics_estimated.orientation
    a2 = quaternion_to_euler(w2.w_val,w2.x_val,w2.y_val,w2.z_val)
    r3 = client.getMultirotorState(vehicle_name="UAV3").kinematics_estimated.position
    w3 = client.getMultirotorState(vehicle_name="UAV3").kinematics_estimated.orientation
    a3 = quaternion_to_euler(w3.w_val,w3.x_val,w3.y_val,w3.z_val)

    v1 = client.getMultirotorState(vehicle_name="UAV1").kinematics_estimated.linear_velocity
    v2 = client.getMultirotorState(vehicle_name="UAV2").kinematics_estimated.linear_velocity
    v3 = client.getMultirotorState(vehicle_name="UAV3").kinematics_estimated.linear_velocity
#    pos1 = pos1.parsed
#    q1 = np.array([pos1['position']['x_val'], pos1['position']['y_val'], pos1['position']['z_val']])
#    qo1 = np.array([pos1['orientation']['w_val'], pos1['orientation']['x_val'], pos1['orientation']['y_val'], pos1['orientation']['z_val']])
    #Add controller
    r1_cap = np.array([r1.x_val + pos0[0,0] - R[0,0], r1.y_val + pos0[0,1] - R[0,1], r1.z_val + pos0[0,2] - R[0,2]])
    r2_cap = np.array([r2.x_val + pos0[1,0] - R[1,0] , r2.y_val + pos0[1,1] - R[1,1] , r2.z_val + pos0[1,2] - R[1,2]])
    r3_cap = np.array([r3.x_val + pos0[2,0] - R[2,0] , r3.y_val + pos0[2,1] - R[2,1] , r3.z_val + pos0[2,2] - R[2,2]])
    
    v1_cap = np.array([v1.x_val- V[0,0], v1.y_val- V[0,1], v1.z_val - V[0,2]])
    v2_cap = np.array([v2.x_val- V[1,0] , v2.y_val- V[1,1], v2.z_val- V[1,2]])
    v3_cap = np.array([v3.x_val- V[2,0] , v3.y_val- V[2,1], v3.z_val- V[2,2]])
    
#    u1x = 0.333*(v2_cap[0] + v3_cap[0] + vt[0] -0.5*((r1_cap[0] - rt[0]) + (r1_cap[0] - r2_cap[0]) + (r1_cap[0] - r3_cap[0])))
#    u2x = 0.5*(v1_cap[0] + v3_cap[0] -0.5*((r2_cap[0] - r1_cap[0]) + (r2_cap[0] - r3_cap[0])))
#    u3x = 0.5*(v1_cap[0] + v2_cap[0] -0.5*((r3_cap[0] - r1_cap[0]) + (r3_cap[0] - r2_cap[0])))
#
#    u1y = 0.333*(v2_cap[1] + v3_cap[1] + vt[1] -0.5*((r1_cap[1] - rt[1]) + (r1_cap[1] - r2_cap[1]) + (r1_cap[1] - r3_cap[1])))
#    u2y = 0.5*(v1_cap[1] + v3_cap[1] -0.5*((r2_cap[1] - r1_cap[1]) + (r2_cap[1] - r3_cap[1])))
#    u3y = 0.5*(v1_cap[1] + v2_cap[1] -0.5*((r3_cap[1] - r1_cap[1]) + (r3_cap[1] - r2_cap[1])))    
#   
#    if S == 3:
##        client.rotateByYawRateAsync(-pi*0.25*sat(a1[2], -pi*0.5, pi*0.5), dt, vehicle_name = "UAV1")
##        client.rotateByYawRateAsync(-pi*0.25*sat(a2[2], -pi*0.5, pi*0.5), dt, vehicle_name = "UAV2")
##        client.rotateByYawRateAsync(-pi*0.25*sat(a3[2], -pi*0.5, pi*0.5), dt, vehicle_name = "UAV3")
##        print('In sequence 3')
##        print(a1[2]*180/pi)
##        print(a2[2]*180/pi)
##        print(a3[2]*180/pi)
#        f1 = client.moveOnPathAsync([airsim.Vector3r(-3*math.cos(0),3*math.sin(0)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.125),3*math.sin(pi*0.125)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.25),3*math.sin(pi*0.25)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.375),3*math.sin(pi*0.375)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.5),3*math.sin(pi*0.5)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.5) + 3,3*math.sin(pi*0.5)+120,-5)],8, dt,vehicle_name = "UAV1")
#        f1.join()
#        f2 = client.moveOnPathAsync([airsim.Vector3r(-3*math.cos(0),3*math.sin(0)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.125),3*math.sin(pi*0.125)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.25),3*math.sin(pi*0.25)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.375),3*math.sin(pi*0.375)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.5),3*math.sin(pi*0.5)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.5) + 1.5,3*math.sin(pi*0.5)+120,-5)],8, dt,vehicle_name = "UAV2")
#        f2.join()
#        f3 = client.moveOnPathAsync([airsim.Vector3r(-3*math.cos(0),3*math.sin(0)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.125),3*math.sin(pi*0.125)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.25),3*math.sin(pi*0.25)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.375),3*math.sin(pi*0.375)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.5),3*math.sin(pi*0.5)+120,-5),airsim.Vector3r(-3*math.cos(pi*0.5),3*math.sin(pi*0.5)+120,-5)],8, dt,vehicle_name = "UAV3")
#        f3.join()
#    else:
    u1x = -0.333*alpha(t,t_0)*((r1_cap[0] - rt[0]) + (r1_cap[0] - r2_cap[0]) + (r1_cap[0] - r3_cap[0]))
    u2x = -0.5*alpha(t,t_0)*((r2_cap[0] - r1_cap[0]) + (r2_cap[0] - r3_cap[0]))
    u3x = -0.5*alpha(t,t_0)*((r3_cap[0] - r1_cap[0]) + (r3_cap[0] - r2_cap[0]))

    u1y = -0.333*alpha(t,t_0)*((r1_cap[1] - rt[1]) + (r1_cap[1] - r2_cap[1]) + (r1_cap[1] - r3_cap[1]))
    u2y = -0.5*alpha(t,t_0)*((r2_cap[1] - r1_cap[1]) + (r2_cap[1] - r3_cap[1]))
    u3y = -0.5*alpha(t,t_0)*((r3_cap[1] - r1_cap[1]) + (r3_cap[1] - r2_cap[1]))    
    
    f1 = client.moveByVelocityZAsync(u1x,u1y, -5,dt, vehicle_name = "UAV1") # Motion at fixed altitude
    f1.join()
    
    f2 = client.moveByVelocityZAsync(u2x,u2y, -5,dt, vehicle_name = "UAV2") # Motion at fixed altitude
    f2.join()
    
    f3 = client.moveByVelocityZAsync(u3x,u3y, -5,dt, vehicle_name = "UAV3") # Motion at fixed altitude
    f3.join()
    
#    u1n = np.array([[u1x],[u1y]])
#    R = rot(g)
#    u1 = np.array([R[0,0]*u1n[0] + R[0,1]*u1n[1],R[1,0]*u1n[0] + R[1,1]*u1n[1]])
#    u1x = float(u1[0])
#    u1y = float(u1[1])
#    
#    u2n = np.array([[u2x],[u2y]])
#    R = rot(g)
#    u2 = np.array([R[0,0]*u2n[0] + R[0,1]*u2n[1],R[1,0]*u2n[0] + R[1,1]*u2n[1]])
#    u2x = float(u2[0])
#    u2y = float(u2[1])
#    
#    u3n = np.array([[u3x],[u3y]])
#    R = rot(g)
#    u3 = np.array([R[0,0]*u3n[0] + R[0,1]*u3n[1],R[1,0]*u3n[0] + R[1,1]*u3n[1]])
#    u3x = float(u3[0])
#    u3y = float(u3[1])
    C1 = np.linalg.norm(r1_cap - rt)
    C2 = np.linalg.norm(r2_cap - rt)
    C3 = np.linalg.norm(r3_cap - rt)
#    print([C1, C2, C3])
    # Add initial coordinates
#    qd = q1 + pos0[0,:]
#        
#        # 3D and 2D state vector
#    q[3*1:3*1+3] =  qd.copy()
#    qxy[2*1:2*1+2] = np.array([qd[0], qd[1]])  
#    qo[4*1:4*1+4] =  qo1.copy()
#    
    
    # Calculate distributed control
#    dqxy = np.zeros(2*numUAV) # Preallocate vectors
#    for i in range(numUAV):
#        if flag[i] == 1:
#            # 3D and 2D state vector
#            qi = Tw[i*numUAV: (i+1)*numUAV, :].flatten() 
#            qxyi = Tw[i*numUAV: (i+1)*numUAV, 0:2].flatten() 
#            
#            # Control
#            dqxyi = A[i*2: i*2+2,:].dot(qxyi)
#    dqxy[2*1:2*1+2] = gain * dqxyi
#    if save == 1:
#        np.save("SavedData/u"+str(itr),dqxy) # Save control
#    
#    
#    # Collision avoidance  # meng.edit('ColAvoid_Ver2_1')    
#    um = meng.ColAvoid_Ver2_1(dqxy.tolist(),qxy.tolist(), dcoll, rcoll, nargout=1)
#    u = np.asarray(um).flatten()     
##    u = dqxy
#    
#    *
    # Saturate velociy control command
#    for i in range(numUAV):
#        # Find norm of control vector for each UAV
#        ui = u[2*i : 2*i+2]
#        vel = np.linalg.norm(ui)
#        if vel > vmax:
#            u[2*i : 2*i+2] = (vmax / vel) * u[2*i : 2*i+2]
#    if save == 1:
#        np.save("SavedData/um"+str(itr),u) # Save modified control
#    
#    
##     Apply control command    
#    for i in range(numUAV):
#        name = "UAV" + str(i+1)
    
    
    
    with open(File, 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        info = {
            "Time": t,
            "Drone1X": r1_cap[0],
            "Drone1Y": r1_cap[1],
            "Drone2X": r2_cap[0],
            "Drone2Y": r2_cap[1],
            "Drone3X": r3_cap[0],
            "Drone3Y": r3_cap[1],
            "Drone1VX": v1_cap[0],
            "Drone1VY": v1_cap[1],
            "Drone2VX": v2_cap[0],
            "Drone2VY": v2_cap[1],
            "Drone3VX": v3_cap[0],
            "Drone3VY": v3_cap[1],
        }

        csv_writer.writerow(info)
#        print(x_value, total_1, total_2)
#    print()

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, "UAV1")
client.armDisarm(False, "UAV2")
client.armDisarm(False, "UAV3")
#client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "UAV1")
#client.enableApiControl(False, "Drone2")


