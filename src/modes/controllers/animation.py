import rospy
import bpy
import Utils, BlenderUtils
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from robo_blender.srv import *
class AnimationControl:

    def read_all_motors_curr_frame(self)
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

    def handle_cmd(self,msg):
        found=False
        for config in self.animation_map:
            found= (msg.command == config['name'])
            if found: break

        if not found:
            print("Animation not found: %s\n",msg.command)
            return AnimateResponse(-1)
        frame_start = int(config["frame_start"])
        frame_stop=int(config["frame_stop"])
        min_secs=float(config["min_secs"])
        print("frame mapped")
        if (frame_start<=0) or (frame_stop<=0):
            print("frame value can't be less than zero")
            return AnimateResponse(-3)
        if frame_start==frame_stop:
            print("No animation range")
            return AnimateResponse(-4)
        if msg.secs==0:
            print("doing fastest time")
            self.animate_bl(frame_start,frame_stop,min_secs)
            return AnimateResponse(1)
        if msg.secs<min_secs:
            print("Duration too low. Minimum duration=%f\n",min_secs)
            return AnimateResponse(-2)
        self.animate_bl(frame_start,frame_stop,msg.secs)
        return AnimateResponse(0)

    def create_new_structure(self):
        self.motor_category={}#dictionary of categories
        for joint_group in self.motors:
            joint_info={}#dictionary of joints
            for joint in joint_group['Joint']:
                joint_info[joint['JointName']]=(joint,[])#tuple of joint data
            self.motor_category[joint_group['JointGroupName']]=joint_info#dictionary of joints

    def __init__(self):
        self.animation_map = Utils.read_yaml("zeno_animation_map.yaml")
        self.motors = Utils.read_yaml("zeno_blender_joints.yaml")
        #self.sub = rospy.Subscriber("AnimationCommand", AnimateCommand, self.handle_cmd)
        self.srvc=rospy.Service("AnimationCommand",animate,self.handle_cmd)

#convert service to action and limit to one action at a time .. mandeep