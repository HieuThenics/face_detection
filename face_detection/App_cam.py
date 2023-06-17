import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GstApp", "1.0")

import numpy as np
from gi.repository import Gst, GstVideo, GstApp
from model import load_model
from crop_object import crop_face
import argparse

predictor = load_model()

global appsrc, height, width


def on_frame_probe(pad, info):
    buffer = info.get_buffer()
    pixle_bytes = 3

    is_mapped, map_info = buffer.map(Gst.MapFlags.READ)

    if is_mapped:
        image_array = np.ndarray((height, width, pixle_bytes), dtype=np.uint8, buffer=map_info.data).copy()
        outputs_frame = crop_face(image_array, predictor)
        gst_buffer = Gst.Buffer.new_wrapped(outputs_frame.astype(np.uint8).tobytes())
        appsrc.emit("push-buffer", gst_buffer)

    return Gst.PadProbeReturn.OK


def main(args):
    Gst.init()
    frame_format = "BGR"
    global appsrc, height, width
    height, width = args['height'], args['width']

    first_pipeline = Gst.Pipeline()
    if not first_pipeline:
        print("first_pipeline was not instantiated")

    second_pipeline = Gst.Pipeline()
    if not second_pipeline:
        print("second first_pipeline was not instantiated")

    source = "v4l2src"
    v4l2src = Gst.ElementFactory.make(source, "local_cam")
    if not v4l2src:
        print("v4l2src was not instantiated")

    videoconvert_0 = Gst.ElementFactory.make("videoconvert", "video_convert0")
    if not videoconvert_0:
        print("videoconvert_0 was not instantiated")

    appsink = Gst.ElementFactory.make("appsink", "app_sink")
    if not appsink:
        print("appsink was not instantiated")

    appsrc = Gst.ElementFactory.make("appsrc", "app_src")
    if not appsrc:
        print("appsrc was not instantiated")

    videoconvert_1 = Gst.ElementFactory.make("videoconvert", "video_convert1")
    if not videoconvert_1:
        print("videoconvert_1 was not instantiated")

    autovideosink = Gst.ElementFactory.make("autovideosink", "auto_video_sink")
    if not autovideosink:
        print("autovideosink was not instantiated")

    appsink.set_property("caps", Gst.Caps.from_string(f"video/x-raw, format={frame_format}"))

    appsrc.set_property("caps", Gst.Caps.from_string(
        f"video/x-raw, framerate=1/30,format={frame_format},height={height}, width={width}"))
    # instructs appsrc that it will deal with timed buffer
    appsrc.set_property('format', Gst.Format.TIME)
    appsrc.set_property('block', True)
    # Set sync = False to avoid late frame drops at the display-sink
    autovideosink.set_property("sync", False)

    first_pipeline.add(v4l2src)
    first_pipeline.add(videoconvert_0)
    first_pipeline.add(appsink)

    second_pipeline.add(appsrc)
    second_pipeline.add(videoconvert_1)
    second_pipeline.add(autovideosink)

    v4l2src.link(videoconvert_0)
    videoconvert_0.link(appsink)

    appsrc.link(videoconvert_1)
    videoconvert_1.link(autovideosink)

    appsink.get_static_pad("sink").add_probe(
        Gst.PadProbeType.BUFFER,
        on_frame_probe
    )

    first_pipeline.set_state(Gst.State.PLAYING)
    second_pipeline.set_state(Gst.State.PLAYING)

    try:
        while True:

            msg = first_pipeline.get_bus().timed_pop_filtered(
                Gst.SECOND,
                Gst.MessageType.EOS | Gst.MessageType.ERROR
            )

            s_msg = second_pipeline.get_bus().timed_pop_filtered(
                Gst.SECOND,
                Gst.MessageType.EOS | Gst.MessageType.ERROR
            )

            if msg:
                text = msg.get_structure().to_string() if msg.get_structure() else ''
                msg_type = Gst.message_type_get_name(msg.type)
                print(f'{msg.src.name}: [{msg_type}] {text}')
                break

            if s_msg:
                text = s_msg.get_structure().to_string() if s_msg.get_structure() else ''
                msg_type = Gst.message_type_get_name(s_msg.type)
                print(f'{s_msg.src.name}: [{msg_type}] {text}')
                break

    finally:
        appsrc.emit("end-of-stream")
        first_pipeline.set_state(Gst.State.NULL)
        second_pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=int, required=True,
                    help="the height of the frame")
    ap.add_argument("--width", type=int, required=True,
                    help="the width of the frame")

    args = vars(ap.parse_args())
    main(args)
