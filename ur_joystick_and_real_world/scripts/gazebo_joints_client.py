import rospy
import kinova_msgs.msg
import time
import threading
from sensor_msgs.msg import JointState

class GazeboJointsClient:
    def __init__(self):
        self.lock = threading.Lock()
        self.theta = []
        self.fingers_positions = []
        self.thread_joints = threading.Thread(target=self.joint_states_listener)
        self.thread_fingers = threading.Thread(target=self.finger_states_listener)
        self.cond = threading.Condition(self.lock)
        self.thread_joints.start()
        self.thread_fingers.start()
        self.timeout = 0.01

    def joint_states_listener(self):
        rospy.Subscriber('/jaco/joint_state', JointState, self.gimme_joints, queue_size=5)
        rospy.spin()

    def finger_states_listener(self):
        rospy.Subscriber('/jaco/joint_state', JointState, self.gimme_fingers, queue_size=5)
        rospy.spin()

    def gimme_joints(self, msg):
        self.lock.acquire()
        current_time = start_time = time.time()
        if msg == []:
            self.cond.wait(self.timeout - current_time + start_time)
        self.theta = [msg.position[1], msg.position[2], msg.position[3],
                      msg.position[4], msg.position[5], msg.position[6]]
        self.lock.release()

    def gimme_fingers(self, msg):
        self.lock.acquire()
        current_time = start_time = time.time()
        if msg == []:
            self.cond.wait(self.timeout - current_time + start_time)
        self.fingers_positions = [msg.position[7], msg.position[8], msg.position[9]]
        self.lock.release()

    def get_joints_states(self):
        if self.theta == []:
            rospy.logerr("no gazebo packets for angles received")
            return self.theta
        self.lock.acquire()
        joints_angles = self.theta
        self.lock.release()
        return joints_angles

    def get_fingers_states(self):
        if self.fingers_positions == []:
            rospy.logerr("no gazebo packets for fingers received")
            return self.fingers_positions
        self.lock.acquire()
        fingers_angles = self.fingers_positions
        self.lock.release()
        return fingers_angles