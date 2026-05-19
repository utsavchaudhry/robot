#!/usr/bin/env python3
"""
Signaling bridge: connects to the WebSocket signaling server as 'robot' and
bridges between WebSocket and ROS topics so webrtc_node can send/receive
SDP and ICE. Without this, webrtc_node's webrtc/signaling_out is never
forwarded to the browser, and the browser never gets an offer.
"""

import asyncio
import json
import logging
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import websockets
except ImportError:
    websockets = None


class SignalingBridge(Node):
    def __init__(self):
        super().__init__('signaling_bridge')

        self.declare_parameter('signaling_port', 8443)
        self.declare_parameter('signaling_host', 'localhost')
        port = int(self.get_parameter('signaling_port').value)
        host = str(self.get_parameter('signaling_host').value)
        self.ws_url = f'ws://{host}:{port}'

        self.to_ws: queue.Queue = queue.Queue()
        self.from_ws: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._ws_thread = None

        self.pub_peer_joined = self.create_publisher(String, 'webrtc/peer_joined', 10)
        self.pub_peer_left = self.create_publisher(String, 'webrtc/peer_left', 10)
        self.pub_signaling_in = self.create_publisher(String, 'webrtc/signaling_in', 10)
        self.sub_signaling_out = self.create_subscription(
            String, 'webrtc/signaling_out', self._on_signaling_out, 10
        )

        self._timer = self.create_timer(0.05, self._drain_from_ws)  # 50 ms

        if websockets is None:
            self.get_logger().error('websockets not installed; signaling bridge disabled')
            return

        self._ws_thread = threading.Thread(target=self._ws_loop, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(f'Signaling bridge: connecting as robot to {self.ws_url} (retries until server is ready)')

    def _on_signaling_out(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('webrtc/signaling_out invalid JSON')
            return
        peer_id = d.get('peer_id')
        typ = d.get('type')
        if not peer_id or not typ:
            return
        if typ == 'offer':
            out = {'type': 'offer', 'sdp': d.get('sdp'), 'target': peer_id}
        elif typ == 'answer':
            out = {'type': 'answer', 'sdp': d.get('sdp'), 'target': peer_id}
        elif typ == 'ice':
            out = {
                'type': 'ice',
                'candidate': d.get('candidate'),
                'sdpMLineIndex': d.get('sdpMLineIndex'),
                'sdpMid': d.get('sdpMid'),
                'target': peer_id,
            }
        elif typ == 'session_rejected':
            out = {
                'type': 'session_rejected',
                'target': peer_id,
                'reason': d.get('reason', 'Session rejected'),
            }
        else:
            return
        self.to_ws.put(out)

    def _drain_from_ws(self):
        try:
            while True:
                m = self.from_ws.get_nowait()
                if m.get('type') == '_error':
                    self.get_logger().error('WebSocket error in signaling bridge')
                    continue
                if m.get('type') == 'registered':
                    continue
                if m.get('type') == 'peer-joined':
                    pid = m.get('peer_id', '')
                    if pid:
                        s = String()
                        s.data = pid
                        self.pub_peer_joined.publish(s)
                        self.get_logger().info(f'Peer joined: {pid}')
                elif m.get('type') == 'peer-left':
                    pid = m.get('peer_id', '')
                    if pid:
                        s = String()
                        s.data = pid
                        self.pub_peer_left.publish(s)
                        self.get_logger().info(f'Peer left: {pid}')
                elif m.get('type') == 'answer':
                    payload = {
                        'type': 'answer',
                        'sdp': m.get('sdp'),
                        'peer_id': m.get('from'),
                    }
                    s = String()
                    s.data = json.dumps(payload)
                    self.pub_signaling_in.publish(s)
                elif m.get('type') == 'offer':
                    # Handle renegotiation offer from browser
                    payload = {
                        'type': 'offer',
                        'sdp': m.get('sdp'),
                        'peer_id': m.get('from'),
                    }
                    s = String()
                    s.data = json.dumps(payload)
                    self.pub_signaling_in.publish(s)
                    self.get_logger().info(f'Forwarding renegotiation offer from {m.get("from")}')
                elif m.get('type') == 'renegotiation_request':
                    # Handle renegotiation request from browser
                    payload = {
                        'type': 'renegotiation_request',
                        'peer_id': m.get('from'),
                    }
                    s = String()
                    s.data = json.dumps(payload)
                    self.pub_signaling_in.publish(s)
                    self.get_logger().info(f'Forwarding renegotiation request from {m.get("from")}')
                elif m.get('type') == 'ice':
                    payload = {
                        'type': 'ice',
                        'candidate': m.get('candidate'),
                        'sdpMLineIndex': m.get('sdpMLineIndex'),
                        'sdpMid': m.get('sdpMid'),
                        'peer_id': m.get('from'),
                    }
                    s = String()
                    s.data = json.dumps(payload)
                    self.pub_signaling_in.publish(s)
        except queue.Empty:
            pass

    def _ws_loop(self):
        async def run():
            backoff = 2.0
            while not self._stop.is_set():
                try:
                    async with websockets.connect(self.ws_url) as ws:
                        await ws.send(json.dumps({'client_id': 'robot'}))
                        backoff = 2.0  # Reset on successful connection
                        while not self._stop.is_set():
                            # send ALL queued messages (not just one)
                            for _ in range(50):
                                try:
                                    msg = self.to_ws.get_nowait()
                                    await ws.send(json.dumps(msg))
                                except queue.Empty:
                                    break
                            # recv up to 10 messages with short timeout
                            for _ in range(10):
                                try:
                                    raw = await asyncio.wait_for(ws.recv(), timeout=0.005)
                                    data = json.loads(raw)
                                    self.from_ws.put(data)
                                except asyncio.TimeoutError:
                                    break
                                except json.JSONDecodeError:
                                    pass
                except Exception as e:
                    err_str = str(e)
                    # Connection refused at startup (server not ready yet): retry quietly, no _error
                    is_refused = '111' in err_str or ('Connect' in err_str and 'failed' in err_str) or 'Connection refused' in err_str
                    if not self._stop.is_set():
                        if is_refused:
                            self.get_logger().info(f'Signaling server not ready, retry in {backoff:.0f}s')
                        else:
                            logging.getLogger('signaling_bridge').warning('WebSocket: %s; reconnect in %.0fs', e, backoff)
                    if not is_refused:
                        self.from_ws.put({'type': '_error'})
                if not self._stop.is_set():
                    try:
                        await asyncio.sleep(backoff)
                        backoff = min(backoff * 2, 30.0)
                    except asyncio.CancelledError:
                        break

        asyncio.run(run())

    def destroy_node(self, *args, **kwargs):
        self._stop.set()
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    node = SignalingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
