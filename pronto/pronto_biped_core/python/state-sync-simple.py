#!/usr/bin/python
# A very simple process to combine the floating base estimate
# with the kinematics and output the combined message
# input: POSE_BODY, FORCE_TORQUE and CORE_ROBOT_STATE output: EST_ROBOT_STATE
#

import os,sys
import lcm
import time
from lcm import LCM
from math import *
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

home_dir =os.getenv("DRC_BASE")
if (home_dir is not None):
  #print home_dir
  sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
  sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

from bot_core.pose_t import pose_t
from bot_core.robot_state_t import robot_state_t
from bot_core.joint_state_t import joint_state_t
from bot_core.vector_3d_t import vector_3d_t
from bot_core.position_3d_t import position_3d_t
from bot_core.twist_t import twist_t
from bot_core.quaternion_t import quaternion_t
from bot_core.twist_t import twist_t
from bot_core.force_torque_t import force_torque_t

########################################################################################
def timestamp_now (): return int (time.time () * 1000000)

global core_robot_state

core_robot_state = joint_state_t()


def on_core_robot_state(channel, data):
  global core_robot_state
  core_robot_state = joint_state_t.decode(data)

def on_pose_body(channel, data):
  global core_robot_state, joint_names
  if (core_robot_state.num_joints==0):
    return

  m = pose_t.decode(data)

  o = robot_state_t()
  o.utime = m.utime
  o.num_joints = core_robot_state.num_joints
  o.joint_name = core_robot_state.joint_name
  o.joint_position = core_robot_state.joint_position
  o.joint_velocity = core_robot_state.joint_velocity
  o.joint_effort = core_robot_state.joint_effort


  nrot = quaternion_t()
  nvec = vector_3d_t()
  p = position_3d_t()
  p.rotation = nrot
  p.translation = nvec
  o.pose = p
 
  t = twist_t()
  t.linear_velocity = nvec
  t.angular_velocity = nvec
  o.twist = t

  o.pose.translation.x =m.pos[0];
  o.pose.translation.y =m.pos[1];
  o.pose.translation.z =m.pos[2];
  o.pose.rotation.w = m.orientation[0];
  o.pose.rotation.x = m.orientation[1];
  o.pose.rotation.y = m.orientation[2];
  o.pose.rotation.z = m.orientation[3];

  ft = force_torque_t()
  o.force_torque = ft

  lc.publish("EST_ROBOT_STATE",o.encode())  

####################################################################
lc = lcm.LCM()
print "started"


sub1 = lc.subscribe("VAL_CORE_ROBOT_STATE|CORE_ROBOT_STATE", on_core_robot_state)
sub2 = lc.subscribe("POSE_BODY", on_pose_body)

while True:
  lc.handle()
