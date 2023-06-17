import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GstApp", "1.0")

import numpy as np
from fractions import Fraction
from gi.repository import Gst, GstVideo, GstApp

rtsp = "rtsp://demo:demo@ipvmdemo.dyndns.org:5541/onvif-media/media.amp?profile=profile_1_h264&sessiontimeout=60&streamtype=unicast"
Gst.init()
source = Gst.ElementFactory.make("rtspsrc", "rtsp_src")

if not source:
    print("source was not instantiated")
source.set_property("location", rtsp)
#source.add_pad()
pads = source.get_static_pad("sink")


array = np.zeros((20,20,3), dtype=np.float16)
#array=1
if isinstance(array, np.ndarray):

    print(isinstance(array, np.ndarray))
    #exit()
print('end')