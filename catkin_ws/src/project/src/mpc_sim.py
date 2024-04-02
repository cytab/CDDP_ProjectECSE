
import rospy
from cddp import CDDP
import numpy as np
from math import atan2, sin, cos, pi, sqrt
from numpy import linalg as LA
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan
import time
from scipy.integrate import odeint
from scipy.optimize import minimize
from systems import Car
from constraints import CircleConstraintForCar
import matplotlib.pyplot as plt
import threading 

global path_pub
global opti_path_pub
path = Path()
optipath = Path()
def quat_to_yaw(quater):
         
    w = quater.w
    x = quater.x
    y = quater.y
    z = quater.z

    raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

    return raw

def yaw_to_quat(yaw):
        
    w = cos(yaw/2)
    x = 0
    y = 0
    z = sin(yaw/2)

    quat = Quaternion(w=w, x=x, y=y, z=z)

    return quat

def dist(s):
    dis = sqrt((s[0]-3)**2 + (s[1]-3)**2)
    print(dis)
    return dis

def callback_odom(odom):
    global state
    xr = odom.pose.pose.position.x
    yr = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation
    v = odom.twist.twist.linear.x
    raw = quat_to_yaw(qz)
    
    xr = xr #- 0.1*v*sin(raw)
    yr = yr #- 0.1*v*cos(raw)
    state = np.array([xr, yr, raw])
    path_publisher(path_pub ,data=odom)
    
def path_publisher(pub, data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    pub.publish(path)

def optimal_path_pub(data):
    optiPath = Path()
    optiPath.header.seq = 0
    optiPath.header.stamp = rospy.get_rostime()
    optiPath.header.frame_id = "odom"

    for i in range(0, data.x_trajectories.shape[1]-1, 5):
        ps = PoseStamped()
        ps.pose.position.x = data.x_trajectories[0, i]
        ps.pose.position.y = data.x_trajectories[1, i]
        ps.pose.orientation.w = data.x_trajectories[2, i]

        optiPath.poses.append(ps)
    return optiPath
    
    

state = np.array([0.0, 0.0, 0.0])
system = Car()
system.set_cost(np.zeros((3, 3)), 0.05*np.identity(2))
Q_f = np.identity(3)
Q_f[0, 0] = 50
Q_f[1, 1] = 50
Q_f[2, 2] = 50
#Q_f[3, 3] = 10
system.set_final_cost(Q_f)
system.set_goal(np.array([2, 4, np.pi/2]))

global move 
move = Twist()
global solver
solver = CDDP(system, np.zeros(3), horizon=250)

#plt.figure()
#plt.plot(range(len(solver.u_trajectories[0,:])), solver.u_trajectories[0,:], label="vitesse ang")
#plt.plot(range(len(solver.u_trajectories[0,:])), solver.u_trajectories[1,:], label="vitesse lin")
#plt.show()
try:
    
    constraint = CircleConstraintForCar(np.ones(2), 0.5, system)
    constraint2 = CircleConstraintForCar(np.array([2, 2]), 0.85, system)
    for i in range(20):
        solver.backward_pass()
        solver.forward_pass()
    solver.add_constraint(constraint)
    solver.add_constraint(constraint2)
    solver.system.draw_trajectories(solver.x_trajectories)
    for i in range(20):
            solver.backward_pass()
            solver.forward_pass()
    system.set_goal(np.array([3, 3, np.pi/2]))
    # node initialization
    rospy.init_node('control_node')

    # subscriber odometry
    rate = rospy.Rate(10)
    sub1 = rospy.Subscriber('/odom', Odometry, callback_odom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    opti_path_pub = rospy.Publisher('/path_opti', Path, queue_size=10)
    
    waypoint = optimal_path_pub(data=solver)
    opti_path_pub.publish(waypoint)
    
    while not rospy.is_shutdown():
        solver.reset_trajectory_size(state)
        print(state)
        for i in range(10):
            solver.backward_pass()
            solver.forward_pass()
        waypoint = optimal_path_pub(solver)
        opti_path_pub.publish(waypoint)
        #solver.system.draw_trajectories(solver.x_trajectories)
       # print('we good')
        if dist(s=state) <= 0.01 :
            move.linear.x = 0
            move.linear.y = 0
            move.linear.z = 0
            move.angular.z = 0
            pub.publish(move)
        else:
            #print(move)       
            move.linear.x = solver.u_trajectories[0,0]
            move.angular.z = solver.u_trajectories[1,0]
            pub.publish(move)
        rate.sleep()
    
except rospy.ROSInterruptException:
    pass