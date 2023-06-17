import time

import gi
from fractions import Fraction
import numpy as np

gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('GstApp', '1.0')

from gi.repository import Gst, GstVideo, GstApp


def fraction_to_str(fraction: Fraction) -> str:
    """Converts fraction to str"""
    return '{}/{}'.format(fraction.numerator, fraction.denominator)


Gst.init()

frame_format = "BGR"
GST_VIDEO_FORMAT = GstVideo.VideoFormat.from_string("BGR")
fps = 30
FPS = Fraction(fps)
width, height = "640", "480"
pipeline = Gst.Pipeline()

appsrc = Gst.ElementFactory.make('appsrc', 'app_src')
if not appsrc:
    print("appsrc was not instantiated")

queue = Gst.ElementFactory.make('queue', 'video_queue')
if not queue:
    print("queue element was not instantiated")

video_convert = Gst.ElementFactory.make('videoconvert', 'video_convert')
if not video_convert:
    print("videoconvert element was not instantiated")

auto_video_sink = Gst.ElementFactory.make('autovideosink', 'auto_video_sink')
if not auto_video_sink:
    print("autovideosink element was not instantiated")

appsrc.set_property("caps", Gst.Caps.from_string(
    f"video/x-raw,format={frame_format},width={width},height={height}"))
appsrc.set_property('format', Gst.Format.TIME)
# appsrc.set_property('block', True)
auto_video_sink.set_property("sync", False)

pipeline.add(appsrc)
# pipeline.add(queue)
pipeline.add(video_convert)
pipeline.add(auto_video_sink)

appsrc.link(video_convert)
# queue.link(video_convert)
video_convert.link(auto_video_sink)

pipeline.set_state(Gst.State.PLAYING)

try:
    while True:

        array = np.random.randint(low=0, high=255, size=(int(height), int(width), 3), dtype=np.uint8)
        gst_buffer = Gst.Buffer.new_wrapped(array.tobytes())
        appsrc.emit("push-buffer", gst_buffer)


        # pts = 0  # buffers presentation timestamp
        # duration = 10 ** 9 / (FPS.numerator / FPS.denominator)  # frame duration
        # for _ in range(1):
        #     array = np.random.randint(low=0, high=255, size=(int(height), int(width), 3), dtype=np.uint8)
        #     gst_buffer = Gst.Buffer.new_wrapped(array.tobytes())
        #     print(pts)
        #     pts += duration
        #     gst_buffer.pts = pts
        #     gst_buffer.duration = duration
        #
        # appsrc.emit("push-buffer", gst_buffer)

        msg = pipeline.get_bus().timed_pop_filtered(
            Gst.SECOND,
            Gst.MessageType.EOS | Gst.MessageType.ERROR
        )
        if msg:
            text = msg.get_structure().to_string() if msg.get_structure() else ''
            msg_type = Gst.message_type_get_name(msg.type)
            print(f'{msg.src.name}: [{msg_type}] {text}')
            print('die')

            break

    appsrc.emit("end-of-stream")

except Exception as e:
    print(e)

finally:
    pipeline.set_state(Gst.State.NULL)
