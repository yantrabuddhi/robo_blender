import mathutils
from ros_animate.msg import animate_command
#map zeno animation key-frames

#anim=String("wave")
#duration=float(seconds) 
#anim_map= int(start frame),int(stop_frame),float(min_time) 
#"move_1_step_forward"->int(frame_start),int(frame_stop),float(duration),float(min_duration)
#ZenoAnimationSource


class ZenoAnimationSource:
    #anim_map = {'stand_init':[1,24,1.0],
    #'move_1_step_forward_from_stand':[24,72,2.0],
    #'move_1_step_forward':[23,74,2.0],
    #'move_1_step_forward_to_stand':[74,98,1.0]
    #}
    def process(self, msg, config):
        iframe = config["frame_info"].split(":")
        frames=(int(iframe[0]),int(iframe[1]),float(iframe[2]))
        if (msg.anim not in config):
			print("Animation not found: %s\n",msg.anim)
		    return(-1,-1,0.0,0.0)
        if (msg.duration<=frames[2]):
			print("Duration too low. Minimum duration=%f\n",frames[2])
		    return(-2,-2,0.0,0.0)
        frame_play=(frames[0],frames[1],msg.duration,frames[2])
        print("frame mapped")
        return frame_play
    def msgclass(self):
        return animate_command

processors["ZenoAnimationSource"] = ZenoAnimationSource()
