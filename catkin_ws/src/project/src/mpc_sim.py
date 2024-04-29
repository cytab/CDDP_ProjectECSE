
import rospy
from cddp import CDDP
import tf
import numpy as np
from math import atan2, sin, cos, pi, sqrt
from numpy import linalg as LA
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import time
from scipy.integrate import odeint
from scipy.optimize import minimize
from systems import Car
from constraints import CircleConstraintForCar
import matplotlib.pyplot as plt
import threading 
from fast_obstacle_avoidance.utils import laserscan_to_numpy
#from fast_obstacle_avoidance.laserscan_utils import reset_laserscan
from fast_obstacle_avoidance.obstacle_avoider import FastLidarAvoider
from fast_obstacle_avoidance.obstacle_avoider import SampledClusterAvoider


global path_pub
global opti_path_pub
global command_arrow_marker
global command_marker
global fast_avoider
path = Path()
optipath = Path()
All_TRajectories = []
fistr_trajectory = []
odom_Trajectories_x = []
odom_Trajectories_y = [] 
command_marker = Marker()
command_marker.header.frame_id = 'odom'
command_marker.type = Marker.ARROW
command_marker.action = Marker.ADD
command_marker.scale = Vector3(x=0.08, y=0.008, z=0.008)
command_marker.color.a = 1.0
command_marker.color.r = 1.0
command_marker.color.g = 0.0
command_marker.color.b = 0.0


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
    #fast_avoider.update_reference_direction()
    path_publisher(path_pub ,data=odom)
    command_arrow_pub(pub=command_arrow_marker, state=state, solver=solver) 
    
def callback_laser(msg):
    #print(len(msg.ranges))
    dataPoints = laserscan_to_numpy(msg=msg)
    fast_avoider.update_laserscan(dataPoints, in_robot_frame=False)
    #fast_avoider.update_reference_direction(datapoints=dataPoints, in_robot_frame=False)
    
    
def path_publisher(pub, data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    odom_Trajectories_x.append(data.pose.pose.position.x)
    odom_Trajectories_y.append(data.pose.pose.position.y)
    path.poses.append(pose)
    pub.publish(path)

def optimal_path_pub(data):
    optiPath = Path()
    optiPath.header.seq = 0
    optiPath.header.stamp = rospy.get_rostime()
    optiPath.header.frame_id = "odom"
    All_TRajectories.append(solver)
    for i in range(0, data.x_trajectories.shape[1]-1, 5):
        ps = PoseStamped()
        ps.header.stamp = rospy.get_rostime()
        ps.header.frame_id = "odom"
        ps.pose.position.x = data.x_trajectories[0, i]
        ps.pose.position.y = data.x_trajectories[1, i]
        ps.pose.orientation.w = 0#data.x_trajectories[2, i]

        optiPath.poses.append(ps)
    return optiPath
    
def command_arrow_pub(pub, state, solver):
     # Create arrow marker
    command_marker.action = Marker.ADD
    command_marker.header.stamp = rospy.Time.now()
    # Set arrow pose and orientation based on robot speed and orientation
    command_marker.pose.position.x = state[0]
    command_marker.pose.position.y = state[1]
    command_marker.pose.position.z = 0.192
    
    if solver.u_trajectories[1,0] <= 0.03 and solver.u_trajectories[1,0] > -0.03:
        w = solver.u_trajectories[1,0]*(180/np.pi) +90 #(atan2((solver.u_trajectories[0,0]*sin(solver.u_trajectories[1,0])), (solver.u_trajectories[0,0]*cos(solver.u_trajectories[1,0]))))*(180/np.pi)
    else:
        w = solver.u_trajectories[1,0]*(180/np.pi) 
    quat = tf.transformations.quaternion_from_euler(0, 0, w)
    command_marker.pose.orientation.x = quat[0]
    command_marker.pose.orientation.y = quat[1]
    command_marker.pose.orientation.z = quat[2]
    command_marker.pose.orientation.w = quat[3]
    
    pub.publish(command_marker)
    
def command_pose_pub(pub, state, solver):
    ps = PoseStamped()
    ps.pose.position.x = state[0]
    ps.pose.position.y = state[1]
    w = solver.u_trajectories[1,0]*(180/np.pi) 
    quat = tf.transformations.quaternion_from_euler(0, 0, w)
    ps.pose.orientation.x = quat[0]
    ps.pose.orientation.y = quat[1]
    ps.pose.orientation.z = quat[2]
    ps.pose.orientation.w = quat[3]

    pub.publish(ps)
    
def command_to_vxy(solver, state):
    theta = solver.system.dt*solver.u_trajectories[1,0] #+ state[2]
    initial_velocity = np.array([(solver.u_trajectories[0,0]*cos(theta)), (solver.u_trajectories[0,0]*sin(theta))])
    return initial_velocity

def vxy_to_command(modulated):
    theta = atan2(modulated[1], modulated[0])
    v = sqrt(modulated[0]**2 + modulated[1]**2)
    return np.array([v, theta])

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
solver = CDDP(system, np.zeros(3), horizon=200)


try:
    
    constraint = CircleConstraintForCar(np.ones(2), 0.5, system)
    constraint2 = CircleConstraintForCar(np.array([2, 2]), 0.8, system)
    for i in range(40):
        solver.backward_pass()
        solver.forward_pass()
    solver.add_constraint(constraint)
    solver.add_constraint(constraint2)
    solver.system.draw_trajectories(solver.x_trajectories)
    
    system.set_goal(np.array([3, 3, np.pi/2]))
    for i in range(50):
            solver.backward_pass()
            solver.forward_pass()
            
    plt.plot(solver.x_trajectories[0,:], solver.x_trajectories[1,:], color='red')     
    plt.show()  
    
    # node initialization
    rospy.init_node('control_node')

    # subscriber odometry
    rate = rospy.Rate(6.25)
    sub1 = rospy.Subscriber('/odom', Odometry, callback_odom)
    scan_sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    opti_path_pub = rospy.Publisher('/path_opti', Path, queue_size=10)
    command_arrow_marker = rospy.Publisher( '/command_arrow_marker', Marker, queue_size=10)
    
    waypoint = optimal_path_pub(data=solver)
    opti_path_pub.publish(waypoint)
    
    # avoider class
    fast_avoider = FastLidarAvoider(control_radius=0.18,weight_max_norm=1e5,weight_factor=0.035*0.0174,weight_power=8,)
    print('Ros loop started :')
    
    while not rospy.is_shutdown():
        solver.reset_trajectory_size(state)
        for i in range(10):
            solver.backward_pass()
            solver.forward_pass()
        waypoint = optimal_path_pub(solver)
        opti_path_pub.publish(waypoint)
        
        if dist(s=state) <= 0.08 :
            move.linear.x = 0
            move.linear.y = 0
            move.linear.z = 0
            move.angular.z = 0
            pub.publish(move)
            plt.plot(All_TRajectories[0].x_trajectories[0,:], All_TRajectories[0].x_trajectories[1,:], color='red')
            plt.plot(odom_Trajectories_x, odom_Trajectories_y, color='green')
            plt.show()
        else:
            #command_arrow_pub(pub=command_arrow_marker, state=state, solver=solver)
            initial_velocity = command_to_vxy(solver=solver, state=state)
            modulated_velocity = fast_avoider.avoid(initial_velocity, position=np.array([state[0], state[1]]))
            modulated_command = vxy_to_command(modulated=modulated_velocity)
            
            # non modulated velocity
            #move.linear.x = solver.u_trajectories[0,0]
            #move.angular.z = solver.u_trajectories[1,0]
            
            # modulated velocity
            move.linear.x = modulated_command[0]
            move.angular.z = modulated_command[1]
            print(modulated_command[0])
            pub.publish(move)
        rate.sleep()
    
except rospy.ROSInterruptException:
    pass