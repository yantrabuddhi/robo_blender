#!/usr/bin/env python  
#import roslib; roslib.load_manifest('dmitry_tracker')


# notes: mandeep
# we compute a motor trajectory

import rospy
import bpy
import math
import threading
import yaml
import importlib

import actionlib

from mathutils import Matrix, Vector
from math import acos, degrees

from bpy.app.handlers import persistent
from std_msgs.msg import Float64
from std_msgs.msg import UInt16MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint.msg

from ros_pololu_servo.msg import servo_pololu
from ros_faceshift.msg import *
from ros_animate.msg import *

class robo_blender :
    known_processors_callbacks={'faceshift_source': faceshift_callback,
    'ZenoAnimationSource':zeno_callback
    }
    use_joint_array=True

    def read_motor_config(self, config):
        stream = open(config, 'r')
        self.config = yaml.load(stream)
        
    def read_anim_config(self, config):
        stream = open(config, 'r')
        self.anim = yaml.load(stream)

    def read_input_config(self, config):
        stream = open(config, 'r')
        self.inputs = yaml.load(stream)

    def bind_msg_motors(self):
        #faceshift functionality might be incompatible with animation functionality of zeno, so..
        #send faceshift direct to ros or use only either faceshift or zeno_callback
        #mandeep
        def faceshift_callback(msg):
          #apply processor and do stuff
          processor=detected_processors['faceshift_source']
          for con in self.inputs:
              if (con["processor"]=='faceshift_source'):
                r = processor.process(msg,con)
                binding = con["binding"].split(":")
                #r = processor.process(msg, con)
                if binding[0] == "shapekey":
                    self.set_shape_position(con, r)
                if binding[0] == "bone":
                    self.set_bone_position(con, r)
                self.send_motors()
                
        def zeno_callback(msg):
          processor=detected_processors['ZenoAnimationSource']
          r = processor.process(msg,self.anim)
          pololu_arr,dynamixel_arr=self.animate(r)
          
        processors = {}
        self.detected_processors={}
        #source_listeners = {}
        for processor in self.known_processors:
            filename = os.path.join(os.path.dirname(bpy.data.filepath), "processors/%s.py" % processor)
            exec(compile(open(filename).read(), "processors/%s.py" % processor, 'exec'), globals(), locals())
        self.detected_processors=processors
        for processor in detected_processors:
            rospy.Subscriber(detected_processors[processor], detected_processors[processor].msgclass(), known_processors_callbacks[processor])
        for con in self.inputs:
            name = con["name"]
            binding = con["binding"].split(":")
            source = con["source"].split(":")
            if con["processor"] not in processors:
              print("NOT FOUND: Processor %s for \n input %s \nBinding %s\n", con["processor"],con["name"],con["binding"])

    def send_motors(self):
        for motor in self.config:
            name = motor["name"]
            binding = motor["binding"].split(":")
            if binding[0] == "shapekey":
                self.position_motor(motor, self.get_shape_position(motor))
            if binding[0] == "bone":
                self.position_motor(motor, self.get_bone_position(motor))
                
    def animate(infFrames):
        self.anim_fps=24.0
        frame_start=infFrames[0]
        frame_stop=infFrame[1]
        duration=infFrame[2]
        min_time=infFrame[3]
        if frame_start<=0: return
        pol_arr=[]
        dyn_arr=[]
        motor_category={}
        #loop through animations
        nframes=abs(frame_stop-frame_start)+1
        rt=float(nframes)/duration
        if rt<1: rt=1
        ra=rospy.Rate(rt)
        for x in range (frame_start,frame_stop):
            bpy.context.scene.frame_set(frame=x)
            #call make_motor_arr
            pa,da=self.make_motor_arr()
            pol_arr.append(pa)
            dyn_arr.append(da)
            if not use_joint_array: ra.sleep()
        #make motor messages
        if not use_joint_array: return
        #list positions by motor
        mtrl={}#motor dictionary, positions, speeds list
        pangle=8.0
        speed=0
        for frame1 in dyn_arr:
            for db in frame1:
                mt=db[0]
                cangle=db[1]
                if pangle>6.5:
                   speed=0.0
                else:
                   speed=(cangle-pangle)*rt #rt=rate in float
                pangle=cangle
                jnt=mt["joint_category"]
                if mt['name'] not in mtrl: mtrl[mt['name']=[]
                mtrl[mt['name']].append((jnt,cangle,speed))#current angle, speed
        #get all motor categories
        for conf in self.config:
            if conf['joint_category'] not in motor_category:
                motor_category[conf['joint_category']]=[]
                
        #make trajectory message
        for mtrs in mtrl:
            arr=mtrl[mtrs]
            elem=arr[0]
            motor_category[elem[0]].append((mtrs,arr))

        for cats in motor_category:
            traj=JointTrajectory()
            for mtrs in motor_category[cats]:
                mtr_name=mtrs[0]
                mtr_pts=mtrs[1]
                traj.joint_names.append(mtr_name)
                pos_arr=JointTrajectoryPoint()
                for mtr in mtr_pts:
                    pos_arr.positions.append(mtr[1])
                    pos_arr.velocities.append(mtr[2])
                traj.points.append(pos_arr)
        #send trajectory for the category
        
        ##distribute motors by category, make all frames
        #all_frames=[]
        #for pFrame in dyn_arr:
            #motor_cat={}
            #for mm in motor_category: motor_cat[mm]=[]
            #for db in pFrame:
                #mtr=db[0]
                #val=db[1]
                #motor_cat[mtr['joint_category']].append(db)
            #all_frames.append(motor_cat)
        ##convert all_frames to trajectory message
        #jTraj=JointTrajectory()
        #for aFrame in all_frames:
            #for cat in aFrame:
                #for mtr in aFrame[cat]:
                    #jTraj.joint_names.append(mtr[0][name])
                    ##motor,angle -> curr,prev                 
        #publish motor messages
#inputs=commands linked to armature .. inputs.yaml not important for program
#
    def make_motor_arr(self):
        pololu_arr=[]
        dynamixel_arr=[]
        #moto={}
        for motor in self.config:
            name = motor["name"]
            if name not in moto: moto[name]=[]
            binding = motor["binding"].split(":")
            if binding[0] == "shapekey":
                if motor["type"]=="pololu":
                    pololu_arr.append(motor,self.get_shape_position(motor))
                    #moto[name].append(self.get_shape_position(motor))
                    if not use_joint_array: self.position_motor(motor, self.get_shape_position(motor))
                else:
                    dynamixel_arr.append(motor,self.get_shape_position(motor))
                    #moto[name].append(self.get_shape_position(motor))
                    if not use_joint_array: self.position_motor(motor, self.get_shape_position(motor))
            if binding[0] == "bone":
                if motor["type"]=="pololu":
                    pololu_arr.append(motor,self.get_bone_position(motor))
                    #moto[name].append(self.get_bone_position(motor))
                    if not use_joint_array: self.position_motor(motor, self.get_bone_position(motor))
                else:
                    dynamixel_arr.append(motor,self.get_bone_position(motor))
                    #moto[name].append(self.get_bone_position(motor))
                    if not use_joint_array: self.position_motor(motor, self.get_bone_position(motor))
        return pololu_arr,dynamixel_arr #,moto

    def get_bone_position(self, config):
        binding = config["binding"].split(":")
        bone_parent = binding[1]
        bone = binding[2]
        axis = binding[3]
        cur_pos = self.get_bones_rotation_rad(bone_parent, bone, axis)
        #print("GET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, cur_pos))
        return cur_pos

    def set_bone_position(self, config, position):
        binding = config["binding"].split(":")
        bone_parent = binding[1]
        bone = binding[2]
        axis = binding[3]
        bpy.data.objects[bone_parent].pose.bones[bone].rotation_mode = 'XYZ'
        rot = bpy.data.objects[bone_parent].pose.bones[bone].rotation_euler

#        print("AXIS %s" % axis)

        if axis == "x":
            rot[0] = position * -1
        if axis == "y":
            rot[1] = position
        if axis == "z":
            rot[2] = position
        bpy.data.objects[bone_parent].pose.bones[bone].rotation_euler = rot
        #print("SET BONE: %s -> %s : %s = %s" % (bone_parent, bone, axis, rot))
        
    def get_shape_position(self, config):
        binding = config["binding"].split(":")
        shape_parent = binding[1]
        shape = binding[2]
        position = bpy.data.meshes[shape_parent].shape_keys.key_blocks[shape].value
        #print("GET SHAPE: %s -> %s = %s" % (shape_parent, shape, position))
        return position

    def set_shape_position(self, config, position):
        if config["enabled"]:
            binding = config["binding"].split(":")
            shape_parent = binding[1]
            shape = binding[2]
            scale = config["scale"]
            translate = config["translate"]
            #print("SET SHAPE: %s -> %s = %s" % (shape_parent, shape, position))
            if config["invert"]: position = 1 - position
            #comment the line below- replace with position variable .. mandeep 
            bpy.data.meshes[shape_parent].shape_keys.key_blocks[shape].value = position * scale

    def position_motor(self, config, angle):
        #print("POSITION %s" % angle)
        if config["name"] in self.positions:
            cur_pos = self.positions[config["name"]]
            if cur_pos != angle:
                if config["type"] == "pololu": self.position_pololu(config, angle)
                if config["type"] == "dynamixel": self.position_dynamixel(config, angle)
        self.positions[config["name"]] = angle

    def position_pololu(self, config, angle):
        msg = servo_pololu()
        msg.id = int(config["motorid"])
        msg.angle = float(angle * float(config["scale"]) + float(config["translate"]))
        msg.speed = int(config["speed"])
        msg.acceleration = int(config["acceleration"])
        #pub = self.pololus[config["name"]]
        pub = self.pub_pololu
        if config["enabled"]: 
            #print("POLOLU: %s @ %s" % (config["name"], angle))
            pub.publish(msg)

    def position_dynamixel(self, config, angle):
        pub = self.dynamixels[config["name"]]
        if config["enabled"]: 
            #print("DYNAMIXEL: %s @ %s" % (config["name"], angle))
            pub.publish(float(angle))

    def get_pose_matrix_in_other_space(self,mat, pose_bone):
        """ Returns the transform matrix relative to pose_bone's current
        transform space. In other words, presuming that mat is in
        armature space, slapping the returned matrix onto pose_bone
        should give it the armature-space transforms of mat.
        TODO: try to handle cases with axis-scaled parents better.
        """
        rest = pose_bone.bone.matrix_local.copy()
        rest_inv = rest.inverted()
        if pose_bone.parent:
            par_mat = pose_bone.parent.matrix.copy()
            par_inv = par_mat.inverted()
            par_rest = pose_bone.parent.bone.matrix_local.copy()
        else:
            par_mat = Matrix()
            par_inv = Matrix()
            par_rest = Matrix()

        # Get matrix in bone's current transform space
        smat = rest_inv * (par_rest * (par_inv * mat))

        # Compensate for non-local location
        #if not pose_bone.bone.use_local_location:
        # loc = smat.to_translation() * (par_rest.inverted() * rest).to_quaternion()
        # smat.translation = loc

        return smat

    def get_local_pose_matrix(self,pose_bone):
        """ Returns the local transform matrix of the given pose bone.
        """
        return self.get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)

    def getRoll(bone):
        mat = bone.matrix_local.to_3x3()
        quat = mat.to_quaternion()
        if abs(quat.w) < 1e-4:
            roll = pi
        else:
            roll = 2*atan(quat.y/quat.w)
        return roll
    
    def get_bones_rotation(self,armature,bone,axis):
        mat = self.get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
        if axis == 'z':
            return degrees(mat.to_euler().z)
        elif axis == 'y':
            return degrees(mat.to_euler().y)
        elif axis == 'x':
            return degrees(mat.to_euler().x)
        elif axis == 'r':
            return degrees(getRoll(bone))

    def get_bones_rotation_rad(self,armature,bone,axis):
        mat = self.get_local_pose_matrix(bpy.data.objects[armature].pose.bones[bone])
        if axis == 'z':
            return mat.to_euler().z
        elif axis == 'y':
            return mat.to_euler().y
        elif axis == 'x':
            return mat.to_euler().x
        elif axis == 'r':
            return getRoll(bone)

    def execute(self):
        self.positions = {}
        self.dynamixels = {}
        self.pololus = {}

        self.ops = []

        namespace = rospy.get_namespace()
        rospy.init_node('robo_blender', anonymous=True)

        # init pololu
        self.pub_pololu=rospy.Publisher(namespace + 'cmd_pololu', servo_pololu)

        # init dynamixels
        for motor in self.config:
            #if motor["type"] == "pololu" and motor["name"] not in self.pololus:
                #self.pololus[motor["name"]] = pub_pololu
            if motor["type"] == "dynamixels" and motor["name"] not in self.dynamixels:
                self.dynamixels[motor["name"]] = rospy.Publisher(namespace + motor["ros_path"]+'/command', Float64)
                
        self.bind_msg_motors()

        print("ROBO: Started")
        rospy.spin()

print("ROBO: Starting")
robo = robo_blender()
robo.read_anim_config("anim.yaml")
robo.read_motor_config("motors.yaml")
robo.read_input_config("inputs.yaml")
robo.execute()
