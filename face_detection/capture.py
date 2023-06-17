import gi, time

gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GstVideo
import numpy as np


def on_frame_probe(pad, info):
    buf = info.get_buffer()
    caps_format = pad.get_current_caps().get_structure(0)

    height, width = caps_format.get_value('height'), caps_format.get_value('width')
    video_format = GstVideo.VideoFormat.from_string(
        caps_format.get_value('format'))

    print(f'video format: {video_format}')
    image_array = buffer_to_image_array(buf, pad.get_current_caps())

    print(image_array.shape)

    return Gst.PadProbeReturn.OK


def buffer_to_image_array(buf, caps):
    pixel_bytes = 3
    caps_structure = caps.get_structure(0)
    height, width = caps_structure.get_value('height'), caps_structure.get_value('width')
    is_mapped, map_info = buf.map(Gst.MapFlags.READ)

    if is_mapped:
        try:
            print(type(buf))
            image_array = np.ndarray(
                (height, width, pixel_bytes),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()  # extend array lifetime beyond subsequent unmap
            # return preprocess(image_array[:,:,:3]) # RGBA -> RGB

            return image_array
        finally:
            buf.unmap(map_info)


def run():
    frame_format = 'BGR'

    Gst.init()
#rtsp://admin:Abcd1234@113.190.240.99:554/Streaming/Channels/1 rtsp://demo:demo@ipvmdemo.dyndns.org:5541/onvif-media/media.amp?profile=profile_1_h264&sessiontimeout=60&streamtype=unicast
    rtsp = "rtsp://demo:demo@ipvmdemo.dyndns.org:5541/onvif-media/media.amp?profile=profile_1_h264&sessiontimeout=60&streamtype=unicast"
    pipeline = Gst.parse_launch(f'''
        videotestsrc !
        decodebin !
        videoconvert !
        video/x-raw,format={frame_format} !
        fakesink name=s  
    ''')

    pipeline.get_by_name('s').get_static_pad('sink').add_probe(
        Gst.PadProbeType.BUFFER,
        on_frame_probe
    )

    pipeline.set_state(Gst.State.PLAYING)

    try:
        while True:
            time.sleep(0.1)

            msg = pipeline.get_bus().timed_pop_filtered(
                Gst.SECOND,
                Gst.MessageType.EOS | Gst.MessageType.ERROR
            )
            if msg:
                text = msg.get_structure().to_string() if msg.get_structure() else ''
                msg_type = Gst.message_type_get_name(msg.type)
                print(f'{msg.src.name}: [{msg_type}] {text}')
                break
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    run()
