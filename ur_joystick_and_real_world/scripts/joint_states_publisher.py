#!/usr/bin/env python
# license removed for brevity


import pygame
import roslib
import math
import numpy as np
import sys
from direct_kinematics_jaco import direct_kinematics_jaco
from inverse_kinematics_jaco import inverse_kinematics_jaco
from gazebo_joints_client import GazeboJointsClient
import rospy
from sensor_msgs.msg import JointState
from jaco_joints_client import JacoJointsClient


class JointStatePublisher:
    def __init__(self, joystick=None, control_type='gazebo'):

        rospy.init_node('phil_joint_state_publisher', anonymous=True)
        self.rate = 100
        rate = rospy.get_param('~rate', self.rate)
        r = rospy.Rate(rate)
        self.control_type = control_type
        self.joystick = joystick
        self.gazebo_joints_client = GazeboJointsClient()
        if self.control_type == 'real_world':
            self.jaco_joints_client = JacoJointsClient()

        namespace = "jaco_on_table::"
        self.joints = ["jaco_arm_0_joint",
                        "jaco_arm_1_joint",
                        "jaco_arm_2_joint",
                        "jaco_arm_3_joint",
                        "jaco_arm_4_joint",
                        "jaco_arm_5_joint"]
        self.joints = [namespace + "jaco_arm_0_joint",
                       namespace + "jaco_arm_1_joint",
                       namespace + "jaco_arm_2_joint",
                       namespace + "jaco_arm_3_joint",
                       namespace + "jaco_arm_4_joint",
                       namespace + "jaco_arm_5_joint",
                       namespace + "jaco_finger_joint_0",
                       namespace + "jaco_finger_joint_2",
                       namespace + "jaco_finger_joint_4"]

        self.controllers = list()
        self.last_working_pose = None
        # self.joint_states = dict({})

        for controller in self.joints:
            # self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0)
            self.controllers.append(controller)

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/jaco/joint_control', JointState)

        rospy.loginfo("Joint State Publisher started at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    # def controller_state_handler(self, msg):
    #     js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)
    #     self.joint_states[msg.name] = js

    def publish_joint_states(self):
        pygame.event.pump()
        # Construct message & publish joint states

        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        if self.control_type == 'gazebo':
            if self.joystick is None:
                velocities = [0, 0, 0, 0, 0, 0]
            else:
                velocities = [self.joystick.get_axis(0) / 10.0,
                              self.joystick.get_axis(1) / 10.0,
                              self.joystick.get_axis(2) / 10.0, 0, 0, 0]
            latest_joint_positions = self.gazebo_joints_client.get_joints_states()
            latest_fingers_positions = self.gazebo_joints_client.get_joints_states()
            latest_joint_positions += latest_fingers_positions
            if latest_joint_positions == []:
                pass
            else:
                if self.last_working_pose is None:
                    self.last_working_pose = latest_joint_positions
                angular_velocities = self.convert_to_angular_velocities(velocities, latest_joint_positions, 1.0 / self.rate)
                angular_velocities = angular_velocities.reshape(1, 6)[0]
                angular_velocities = np.hstack((angular_velocities, np.array([0, 0, 0])))
                for idx, controller in enumerate(self.controllers):
                    msg.name.append(controller)
                    if abs(angular_velocities[idx]) <= 0.00000001:
                        msg.position.append(self.last_working_pose[idx])
                    else:
                        self.last_working_pose = latest_joint_positions
                        msg.position.append(latest_joint_positions[idx] + velocities[idx] * (1.0 / self.rate))
                        msg.velocity.append(angular_velocities[idx])
                self.joint_states_pub.publish(msg)

        elif self.control_type == 'real_world':
            real_jaco_joints_states = self.jaco_joints_client.get_joints_states()
            real_jaco_fingers_states = self.jaco_joints_client.get_fingers_states()
            if real_jaco_joints_states == [] or real_jaco_fingers_states == []:
                pass
            else:
                real_jaco_joints_states = np.array(real_jaco_joints_states)
                real_jaco_joints_states *= np.array([-1, 1, -1, -1, -1, -1])
                real_jaco_joints_states -= np.array([-np.pi,
                                                     -np.pi / 2.,
                                                     -np.pi / 2.,
                                                     -np.pi,
                                                     -np.pi,
                                                     100 * np.pi / 180.])
                real_jaco_fingers_states = np.array(real_jaco_fingers_states)
                data_to_send = np.hstack([real_jaco_joints_states, real_jaco_fingers_states])
                for idx, controller in enumerate(self.controllers):
                    msg.name.append(controller)
                    if idx > 5:
                        if data_to_send[idx] > 1.:
                            data_to_send[idx] = 1.
                            msg.effort.append(5)
                    msg.position.append(data_to_send[idx])
                #print(real_jaco_joints_states)
                theta_temp = real_jaco_joints_states.reshape(6, 1)
                cartesian_pose, rotation_matrix = direct_kinematics_jaco(theta_temp)
                #print(cartesian_pose)
                self.joint_states_pub.publish(msg)

        else:
            rospy.logerr('wrong inputs, expected "gazebo" or "real_world"')
            print("current input is: ", self.control_type)


            #msg.position.append(joint.position)
            #msg.velocity.append(velocities[idx])
            #msg.effort.append(joint.effort)


    def convert_to_angular_velocities(self, velocities, theta_current, dt):
        velocities = np.array([[velocities[0]],
                                [velocities[1]],
                                [velocities[2]],
                                [velocities[3]],
                                [velocities[4]],
                                [velocities[5]]])
        theta_current = np.array([[theta_current[0]],
                                   [theta_current[1]],
                                   [theta_current[2]],
                                   [theta_current[3]],
                                   [theta_current[4]],
                                   [theta_current[5]]])
        cartesian_pose, rotation_matrix = direct_kinematics_jaco(theta_current)

        next_cartesian_pose = cartesian_pose + np.multiply(velocities[0:3, 0].reshape(3, 1), dt)
        euler_angles = rotationMatrixToEulerAngles(rotation_matrix)
        if velocities[3] == velocities[4] == velocities[5] == 0.:
            next_euler_angles = euler_angles
            next_rotation_matrix = rotation_matrix
        else:
            next_euler_angles = np.array(euler_angles).reshape(3, 1) + np.multiply(velocities[3:6, 0].reshape(3, 1), dt)
            next_rotation_matrix = eulerAnglesToRotationMatrix([next_euler_angles[0, 0],
                                                                next_euler_angles[1, 0],
                                                                next_euler_angles[2, 0]])

        next_theta, success, sol_approx = inverse_kinematics_jaco(next_cartesian_pose, next_rotation_matrix, theta_current)

        # print(np.unwrap(theta_current), "theta_current")
        # print(np.unwrap(next_theta), "next_theta")
        # print(np.subtract(np.unwrap(next_theta), np.unwrap(theta_current)), "theta_diff")
        # print(dt)
        # next_theta[1, 0] *= -1.0
        angular_velocity = np.divide(np.subtract(next_theta, theta_current), dt)
        for idx, velocity in enumerate(angular_velocity.reshape(1, 6)[0]):
            if idx == 0 or idx == 1 or idx == 3 or idx == 4 or idx == 5:
                angular_velocity[idx] = -velocity
            # if velocity*dt > np.pi:
            #     angular_velocity[idx] = velocity - np.pi/dt
            # elif velocity*dt < -np.pi:
            #     angular_velocity[idx] = velocity + np.pi / dt
        return angular_velocity


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):

    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


if __name__ == '__main__':
    try:
        pygame.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        s = JointStatePublisher(joystick)
        rospy.spin()
    except rospy.ROSInterruptException: pass




