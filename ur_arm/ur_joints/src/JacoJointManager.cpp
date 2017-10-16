#ifdef DOXYGEN_SHOULD_SKIP_THIS
/**
   Manages the ur joint names as specified in the URDF file.

   Copyright (C) 2015 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/
#endif

#include <ur_joints/UrJointManager.h>
#include <map>
#include <string>
#include <vector>

#define J0_NAME  "ur_arm_0_joint"
#define J1_NAME  "ur_arm_1_joint"
#define J2_NAME  "ur_arm_2_joint"
#define J3_NAME  "ur_arm_3_joint"
#define J4_NAME  "ur_arm_4_joint"
#define J5_NAME  "ur_arm_5_joint"

#define JF0_NAME  "ur_finger_joint_0"
#define JF1_NAME  "ur_finger_joint_2"
#define JF2_NAME  "ur_finger_joint_4"


#define J0_INIT 4.5532045
#define J1_INIT 4.5506868
#define J2_INIT 0.7038534
#define J3_INIT 5.4655337
#define J4_INIT 1.506298
#define J5_INIT 3.135861
// #define JF_INIT -0.0043633
#define JF_INIT 0


#define HAND_LINK "ur_6_hand_limb"
#define ARM_LINKS "ur_0_baseA", "ur_0_base_limb", "ur_1_shoulder_limb", "ur_2_upperarm_limb",\
    "ur_3_forearm_limb", "ur_4_upperwrist_limb", "ur_5_lowerwrist_limb", "ur_ring_1", \
    "ur_ring_2", "ur_ring_3", "ur_ring_4", "ur_ring_5", "ur_ring_6"
#define FINGER_LINKS "ur_grippers_base_link", "ur_7_gripper_mount_index", "ur_8_gripper_index",\
    "ur_9_gripper_index_tip", "ur_7_gripper_mount_thumb", "ur_8_gripper_thumb", "ur_9_gripper_thumb_tip",\
    "ur_7_gripper_mount_pinkie", "ur_8_gripper_pinkie", "ur_9_gripper_pinkie_tip"

// A value of 5 for MAX_FORCE proved to work with high
// KP values so the hand could just stay up. So shouldn't
// be much higher than that...
#define DEFAULT_MAX_FORCE 5
#define DEFAULT_MAX_FORCE_FINGER 2.5

// maximum velocity for arm joints (rads/sec)
#define DEFAULT_MAX_VEL 1.5
// maximum velocity for arm gripper (rads/sec)
#define DEFAULT_MAX_VEL_FINGER 1.5

UrJointManager::UrJointManager(const std::string& _robot_namespace, bool _readParams):
    arm_components_name_manager::ArmComponentsNameManager(_robot_namespace, _readParams)
{
}

UrJointManager::UrJointManager(const UrJointManager& o):
    arm_components_name_manager::ArmComponentsNameManager(o)
{
}

std::vector<std::string> UrJointManager::getDefaultGripperLinks() const
{
    static std::string arr[] = {FINGER_LINKS};
    int arrSize=sizeof(arr) / sizeof(std::string);

    std::vector<std::string> vec;
    for (int i=0; i<arrSize; ++i) vec.push_back(arr[i]);
    return vec;
}

std::string UrJointManager::getDefaultPalmLink() const
{
    return HAND_LINK;
}

std::vector<std::string> UrJointManager::getDefaultArmLinks() const
{
    static std::string arr[] = {ARM_LINKS, HAND_LINK};
    int arrSize=sizeof(arr) / sizeof(std::string);

    std::vector<std::string> vec;
    for (int i=0; i<arrSize; ++i) vec.push_back(arr[i]);
    return vec;
}

std::vector<std::string> UrJointManager::getDefaultArmJoints() const
{
    std::vector<std::string> _arm_joints;
    _arm_joints.resize(6);
    _arm_joints[0] = J0_NAME;
    _arm_joints[1] = J1_NAME;
    _arm_joints[2] = J2_NAME;
    _arm_joints[3] = J3_NAME;
    _arm_joints[4] = J4_NAME;
    _arm_joints[5] = J5_NAME;
    return _arm_joints;
}

std::vector<std::string> UrJointManager::getDefaultGripperJoints() const
{
    std::vector<std::string> _gripper_joints;
    _gripper_joints.resize(3);
    _gripper_joints[0] = JF0_NAME;
    _gripper_joints[1] = JF1_NAME;
    _gripper_joints[2] = JF2_NAME;
    return _gripper_joints;
}

std::vector<float> UrJointManager::getDefaultArmJointsInitPose() const
{
    std::vector<float> _arm_joint_init;
    _arm_joint_init.resize(6);
    _arm_joint_init[0] = J0_INIT;
    _arm_joint_init[1] = J1_INIT;
    _arm_joint_init[2] = J2_INIT;
    _arm_joint_init[3] = J3_INIT;
    _arm_joint_init[4] = J4_INIT;
    _arm_joint_init[5] = J5_INIT;
    return _arm_joint_init;
}

std::vector<float> UrJointManager::getDefaultGripperJointsInitPose() const
{
    std::vector<float> _gripper_joint_init;
    _gripper_joint_init.resize(3);
    _gripper_joint_init.assign(3, JF_INIT);
    return _gripper_joint_init;
}


std::vector<float> UrJointManager::getDefaultArmJointsMaxVel() const
{
    std::vector<float> _arm_max_vel;
    _arm_max_vel.resize(6);
    _arm_max_vel.assign(6, DEFAULT_MAX_VEL);
    return _arm_max_vel;
}
std::vector<float> UrJointManager::getDefaultArmJointsMaxForce() const
{
    std::vector<float> _arm_max_force;
    _arm_max_force.resize(6);
    _arm_max_force.assign(6, DEFAULT_MAX_FORCE);
    return _arm_max_force;
}

std::vector<float> UrJointManager::getDefaultGripperJointsMaxVel() const
{
    std::vector<float> _gripper_max_vel;
    _gripper_max_vel.resize(3);
    _gripper_max_vel.assign(3, DEFAULT_MAX_VEL_FINGER);
    return _gripper_max_vel;

}

std::vector<float> UrJointManager::getDefaultGripperJointsMaxForce() const
{
    std::vector<float> _gripper_max_force;
    _gripper_max_force.resize(3);
    _gripper_max_force.assign(3, DEFAULT_MAX_FORCE_FINGER);
    return _gripper_max_force;
}

#if 0
bool UrJointManager::GetMaxVals(const std::string& jointName, float& force, float& velocity) const
{
    bool gripper = isGripper(jointName);

    // initialize default values first:
    force = gripper ? DEFAULT_MAX_FORCE_FINGER : DEFAULT_MAX_FORCE;
    velocity = gripper ? DEFAULT_MAX_VEL_FINGER : DEFAULT_MAX_VEL;

    // now read from config file
    const std::vector<std::string>& arm_joints = getArmJoints();

//    ROS_INFO_STREAM("Joint "<<jointName<<" is gripper: "<<gripper);

    if (gripper)
    {
        force = MAX_FINGER_EFFORT;
        velocity = MAX_FINGER_VELOCITY;
    }
    else if (jointName == arm_joints[0])
    {
        force = MAX_ARM_0_EFFORT;
        velocity = MAX_ARM_0_VELOCITY;
    }
    else if (jointName == arm_joints[1])
    {
        force = MAX_ARM_1_EFFORT;
        velocity = MAX_ARM_1_VELOCITY;
    }
    else if (jointName == arm_joints[2])
    {
        force = MAX_ARM_2_EFFORT;
        velocity = MAX_ARM_2_VELOCITY;
    }
    else if (jointName == arm_joints[3])
    {
        force = MAX_ARM_3_EFFORT;
        velocity = MAX_ARM_3_VELOCITY;
    }
    else if (jointName == arm_joints[4])
    {
        force = MAX_ARM_4_EFFORT;
        velocity = MAX_ARM_4_VELOCITY;
    }
    else if (jointName == arm_joints[5])
    {
        force = MAX_ARM_5_EFFORT;
        velocity = MAX_ARM_5_VELOCITY;
    }
    return true;
}
#endif
