#!/usr/bin/env python3
"""
Standalone GStreamer WebRTC sender for local testing (no ROS).
Connects to the signaling server, sends video from videotestsrc or v4l2,
and does SDP/ICE exchange with a browser client.

Run from project root after: source install/setup.bash
  python3 -m robot_webrtc.webrtc_standalone_send --signaling ws://localhost:8443

Optional: --video-device /dev/video0  to use a webcam
          --serve-html 8080           to serve the browser client
"""

import argparse
import asyncio
import json
import logging
import os
import queue
import sys
import threading

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import GLib, Gst, GstWebRTC, GstSdp

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
log = logging.getLogger('webrtc_standalone_send')

try:
    import websockets
except ImportError:
    log.error('websockets required: pip3 install websockets')
    sys.exit(1)


def _resource_dir() -> str:
    """Resource dir: share/robot_webrtc/resource when installed, else ../resource from this file."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('robot_webrtc'), 'resource')
    except Exception:
        return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'resource'))


def ws_thread_func(signaling_url: str, to_ws: queue.Queue, from_ws: queue.Queue, stop: threading.Event):
    async def run():
        try:
            async with websockets.connect(signaling_url) as ws:
                await ws.send(json.dumps({'client_id': 'robot'}))
                log.info('Registered as robot on signaling server')
                while not stop.is_set():
                    # Send queued offer/ice to signaling
                    try:
                        msg = to_ws.get_nowait()
                        await ws.send(json.dumps(msg))
                    except queue.Empty:
                        pass
                    # Receive answer/ice from signaling
                    try:
                        raw = await asyncio.wait_for(ws.recv(), timeout=0.05)
                        data = json.loads(raw)
                        from_ws.put(data)
                    except asyncio.TimeoutError:
                        pass
                    except json.JSONDecodeError:
                        pass
        except Exception as e:
            log.error('WebSocket error: %s', e)
            from_ws.put({'type': '_error'})

    asyncio.run(run())


def main():
    ap = argparse.ArgumentParser(description='Standalone GStreamer WebRTC sender for local testing')
    ap.add_argument('--signaling', default='ws://localhost:8443', help='Signaling WebSocket URL')
    ap.add_argument('--video-device', default='', help='V4L2 device (e.g. /dev/video0). If empty, use videotestsrc')
    ap.add_argument('--serve-html', type=int, default=0, metavar='PORT', help='Serve browser client on PORT (e.g. 8080)')
    ap.add_argument('--no-wait-for-peer', action='store_true', help='Start immediately without waiting for operator')
    ap.add_argument('--wait-timeout', type=int, default=120, metavar='SEC', help='Max seconds to wait for peer (default 120)')
    args = ap.parse_args()

    to_ws = queue.Queue()
    from_ws = queue.Queue()
    stop = threading.Event()

    # Optional: serve HTML for the browser client
    if args.serve_html:
        try:
            from http.server import HTTPServer, SimpleHTTPRequestHandler
        except ImportError:
            log.warning('Cannot import http.server, --serve-html disabled')
        else:
            resdir = _resource_dir()

            class Handler(SimpleHTTPRequestHandler):
                def __init__(self, *a, **k):
                    super().__init__(*a, directory=resdir, **k)
                def log_message(self, *a): pass

            def serve():
                with HTTPServer(('', args.serve_html), Handler) as http:
                    log.info('Serving at http://localhost:%d/webrtc_browser_client.html', args.serve_html)
                    while not stop.is_set():
                        http.handle_request()

            t = threading.Thread(target=serve, daemon=True)
            t.start()

    Gst.init(None)

    use_testsrc = not (args.video_device and os.path.exists(args.video_device))
    if use_testsrc:
        pipeline_str = (
            'webrtcbin name=webrtc bundle-policy=max-bundle '
            'videotestsrc ! video/x-raw,width=640,height=480,framerate=15/1 ! '
            'videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! webrtc.'
        )
        log.info('Using videotestsrc (no camera)')
    else:
        pipeline_str = (
            f'webrtcbin name=webrtc bundle-policy=max-bundle '
            f'v4l2src device={args.video_device} ! video/x-raw,width=640,height=480,framerate=15/1 ! '
            'videoconvert ! queue ! vp8enc deadline=1 ! rtpvp8pay ! queue ! webrtc.'
        )
        log.info('Using v4l2src device=%s', args.video_device)

    pipeline = Gst.parse_launch(pipeline_str)
    if not pipeline:
        log.error('Failed to create pipeline')
        return 1

    webrtc = pipeline.get_by_name('webrtc')
    if not webrtc:
        log.error('webrtcbin not found')
        return 1

    ice_queue = []
    remote_set = [False]  # mutable so closure can set

    def on_offer_created(promise, to_queue):
        promise.wait()
        reply = promise.get_reply()
        if not reply:
            log.error('Failed to create offer')
            return
        offer = reply.get_value('offer')
        p = Gst.Promise.new()
        webrtc.emit('set-local-description', offer, p)
        p.interrupt()
        to_queue.put({'type': 'offer', 'sdp': offer.sdp.as_text()})
        log.info('Sent offer to signaling')

    def on_negotiation_needed(elt, to_queue):
        log.info('Negotiation needed, creating offer')
        promise = Gst.Promise.new_with_change_func(on_offer_created, to_ws)
        webrtc.emit('create-offer', None, promise)

    def on_ice_candidate(elt, mlineindex, candidate, to_queue):
        to_queue.put({'type': 'ice', 'candidate': candidate, 'sdpMLineIndex': mlineindex})
        log.debug('ICE candidate: %s', candidate[:60] if candidate else '')

    webrtc.connect('on-negotiation-needed', on_negotiation_needed, to_ws)
    webrtc.connect('on-ice-candidate', on_ice_candidate, to_ws)

    def drain_signaling():
        try:
            while True:
                msg = from_ws.get_nowait()
                if msg.get('type') == '_error':
                    return False
                if msg.get('type') == 'answer':
                    ret, sdp = GstSdp.SDPMessage.new_from_text(msg['sdp'])
                    if ret != GstSdp.SDPResult.OK:
                        log.error('Invalid SDP answer')
                        continue
                    answer = GstWebRTC.WebRTCSessionDescription.new(
                        GstWebRTC.WebRTCSDPType.ANSWER, sdp
                    )
                    p = Gst.Promise.new()
                    webrtc.emit('set-remote-description', answer, p)
                    p.interrupt()
                    remote_set[0] = True
                    for c in ice_queue:
                        webrtc.emit('add-ice-candidate', c.get('sdpMLineIndex', 0), c.get('candidate', ''))
                    ice_queue.clear()
                    log.info('Set remote description (answer)')
                elif msg.get('type') == 'ice':
                    if remote_set[0]:
                        webrtc.emit('add-ice-candidate', msg.get('sdpMLineIndex', 0), msg.get('candidate', ''))
                    else:
                        ice_queue.append(msg)
        except queue.Empty:
            pass
        return True

    def bus_tick():
        if not drain_signaling():
            main_loop.quit()
            return False
        return True

    # Start WebSocket thread
    th = threading.Thread(target=ws_thread_func, args=(args.signaling, to_ws, from_ws, stop), daemon=True)
    th.start()

    # Give WS time to register
    import time
    time.sleep(0.5)

    # Wait for operator to connect (unless --no-wait-for-peer)
    if not args.no_wait_for_peer:
        log.info('Waiting for operator to connect (up to %ds). Connect browser to signaling server...', args.wait_timeout)
        waited = 0
        peer_joined = False
        while waited < args.wait_timeout and not stop.is_set():
            try:
                msg = from_ws.get(timeout=0.5)
                if msg.get('type') == 'peer-joined':
                    log.info('Operator "%s" connected. Starting pipeline...', msg.get('peer_id', 'unknown'))
                    peer_joined = True
                    break
                elif msg.get('type') == '_error':
                    log.error('WebSocket error during wait')
                    return 1
                elif msg.get('type') == 'registered':
                    # Our own ack; drop it so we don't spin on it
                    pass
                else:
                    from_ws.put(msg)
            except queue.Empty:
                waited += 0.5
        
        if not peer_joined and not stop.is_set():
            log.warning('Timeout waiting for operator. Starting anyway...')
    else:
        log.info('Skipping wait (--no-wait-for-peer). Pipeline will start immediately.')

    pipeline.set_state(Gst.State.PLAYING)
    main_loop = GLib.MainLoop()
    GLib.timeout_add(50, bus_tick)

    log.info('Pipeline playing. Negotiation will start shortly.')
    try:
        main_loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        pipeline.set_state(Gst.State.NULL)

    return 0


if __name__ == '__main__':
    sys.exit(main())
