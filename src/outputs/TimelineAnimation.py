import rospy
import bpy
import Utils, BlenderUtils
import actionlib
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal
# from control_msgs.msg import FollowJointTrajectoryFeedback
# from control_msgs.msg import FollowJointTrajectoryResult

# class FrameMap:
#   """ Represents the position of an animation in the timeline. """
#
#   def __iter__(self):
#     return iter(range(self.frame_start, self.frame_end))
#
#   def set_duration(self, val):
#     self.duration = val
#
#   def get_frame_duration(self):
#     return float(self.frame_start - self.frame_end)/self.duration
#
#   @classmethod
#   def from_string(cls, str):
#     """ Alternative constructor method. """
#     # 'cls' hold the FrameMap class
#     # Asterix below means the list str.split(":") will be expanded into
#     # arguments frame_start, frame_end, min_duration
#     return cls(*str.split(":"))
#
#   def __init__(self, frame_start, frame_end, min_duration):
#     self.frame_start = frame_start
#     self.frame_end = frame_end
#     self.min_duration = min_duration

class TimelineAnimation:
    """
    This output can build and send an animation out, if you give the
    animation's location (FrameMap instance).
    """
    def create_new_structure(self):
        self.motor_category={}#dictionary of categories
        for joint_group in self.motors:
            joint_info={}#dictionary of joints
            for joint in joint_group['Joint']:
                joint_info[joint['JointName']]=(joint,[])#tuple of joint data
            self.motor_category[joint_group['JointGroupName']]=joint_info#dictionary of joints


    def read_all_motors_curr_frame(self):
        for cat in self.motor_category:
            jDict=self.motor_category[cat]
            for joint_name in jDict:
                joint_tuple=jDict[joint_name]
                joint_info=joint_tuple[0]
                joint_arr=joint_tuple[1]
                joint_arr.append(BlenderUtils.get_bones_rotation_rad(joint_info['Armature'],joint_info['Bone'],joint_info['Axis']))

    def animate_bl(self, frame_start,frame_stop,secs):
        self.debug_only=True
        secs_per_frame=secs/float(abs(frame_stop-frame_start))
        self.create_new_structure()
        #loop through animations
        for x in range (frame_start,frame_stop):
            bpy.context.scene.frame_set(frame=x)
            self.read_all_motors_curr_frame()

        all_cat_traj=[]
        #client={}
        #joint_category is same as joint controller
        for cat in self.motor_category:
            jDict=self.motor_category[cat]
            traj=JointTrajectory()
            for joint_name in jDict:
                joint_tuple=jDict[joint_name]
                traj.joint_names.append(joint_name)
                pos_arr=JointTrajectoryPoint()
                pos_arr.time_from_start=secs
                prev_angle=100.0
                joint_arr=joint_tuple[1]
                for mtr in joint_arr:
                    pos_arr.positions.append(mtr)
                    vel=3.142#rad/sec .. initialize to fastest speed
                    if (prev_angle<7.0):
                        vel=(mtr-prev_angle)/secs_per_frame
                    prev_angle=mtr
                    if vel<0.01:#minimum speed in rad/sec
                        vel=0.01
                    pos_arr.velocities.append(vel)
                    if self.debug_only: print("\n motor:%s : pos:%f,speed:%f",joint_name,mtr,vel)
                traj.points.append(pos_arr)
            all_cat_traj.append((cat,traj))
        #send trajectory for the category
        if self.debug_only: return
        self.send_actions(all_cat_traj)

    def send_actions(self, all_trajectories):
        """
        Will send trajectory messages.
        """
        client={}
        for trajectory in all_trajectories:
            cat_topic=trajectory[0]
            trajectory_msg=trajectory[1]
            client[cat_topic] = actionlib.SimpleActionClient(cat_topic, FollowJointTrajectoryAction)
            client[cat_topic].wait_for_server()
            goal=FollowJointTrajectoryActionGoal(trajectory_msg)
            client[cat_topic].send_goal(goal)
        for cat in client:
            client[cat].wait_for_result()

    def __init__(self):
        self.motors = Utils.read_yaml("zeno_blender_joints.yaml")
