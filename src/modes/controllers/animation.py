import rospy
import inputs, outputs, Utils
from robo_blender.srv import *

class AnimationControl:

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
        outputs.TimelineAnimation.animate_bl(frame_start,frame_stop,msg.secs)
        return AnimateResponse(0)

    def __init__(self):
        self.animation_map = Utils.read_yaml("zeno_animation_map.yaml")
        #self.sub = rospy.Subscriber("AnimationCommand", AnimateCommand, self.handle_cmd)
        self.srvc=rospy.Service("AnimationService",Animate,self.handle_cmd)

#convert service to action and limit to one action at a time .. mandeep