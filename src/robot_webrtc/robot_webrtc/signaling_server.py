#!/usr/bin/env python3
"""
Simple WebSocket Signaling Server for WebRTC
Handles SDP offer/answer exchange and ICE candidates between robot and Unity operator
"""

import asyncio
import json
import logging
from typing import Dict, Set
import websockets
from websockets.server import serve, WebSocketServerProtocol


class SignalingServer:
    """WebSocket signaling server for WebRTC peer connection establishment"""

    def __init__(self, host: str = '0.0.0.0', port: int = 8443):
        self.host = host
        self.port = port
        self.clients: Dict[str, WebSocketServerProtocol] = {}
        self.logger = logging.getLogger('SignalingServer')
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    async def register_client(self, websocket: WebSocketServerProtocol, client_id: str):
        """Register a new client connection"""
        self.clients[client_id] = websocket
        self.logger.info(f'Client registered: {client_id} (Total clients: {len(self.clients)})')

    async def unregister_client(self, client_id: str):
        """Unregister a client connection"""
        if client_id in self.clients:
            del self.clients[client_id]
            self.logger.info(f'Client unregistered: {client_id} (Total clients: {len(self.clients)})')

    async def send_to_client(self, client_id: str, message: dict):
        """Send message to specific client"""
        if client_id in self.clients:
            try:
                await self.clients[client_id].send(json.dumps(message))
            except websockets.exceptions.ConnectionClosed:
                self.logger.warning(f'Connection to {client_id} closed')
                await self.unregister_client(client_id)
        else:
            self.logger.warning(f'Client {client_id} not found')

    async def broadcast(self, message: dict, exclude: Set[str] = None):
        """Broadcast message to all clients except excluded ones"""
        if exclude is None:
            exclude = set()
        # Snapshot to avoid "dictionary changed size during iteration" when clients disconnect during broadcast
        disconnected = []
        for client_id, websocket in list(self.clients.items()):
            if client_id not in exclude:
                try:
                    await websocket.send(json.dumps(message))
                except websockets.exceptions.ConnectionClosed:
                    disconnected.append(client_id)
        
        # Clean up disconnected clients
        for client_id in disconnected:
            await self.unregister_client(client_id)

    async def handle_message(self, websocket: WebSocketServerProtocol, client_id: str, message: dict):
        """Handle incoming signaling message"""
        msg_type = message.get('type')
        
        self.logger.info(f'Received {msg_type} from {client_id}')
        
        if msg_type == 'offer':
            # Forward offer to specific peer or broadcast
            target = message.get('target')
            if target:
                await self.send_to_client(target, {
                    'type': 'offer',
                    'sdp': message['sdp'],
                    'from': client_id,
                    'target': target
                })
            else:
                await self.broadcast({
                    'type': 'offer',
                    'sdp': message['sdp'],
                    'from': client_id
                }, exclude={client_id})
                
        elif msg_type == 'answer':
            # Forward answer to specific peer
            target = message.get('target')
            if target:
                await self.send_to_client(target, {
                    'type': 'answer',
                    'sdp': message['sdp'],
                    'from': client_id,
                    'target': target
                })
            else:
                self.logger.warning('Answer message missing target — broadcasting as fallback')
                await self.broadcast({
                    'type': 'answer',
                    'sdp': message['sdp'],
                    'from': client_id
                }, exclude={client_id})
                
        elif msg_type == 'renegotiation_request':
            # Forward renegotiation request to robot
            target = message.get('target')
            if target:
                await self.send_to_client(target, {
                    'type': 'renegotiation_request',
                    'from': client_id
                })
            else:
                self.logger.warning('Renegotiation request missing target')
                
        elif msg_type == 'ice':
            # Forward ICE candidate
            target = message.get('target')
            if target:
                await self.send_to_client(target, {
                    'type': 'ice',
                    'candidate': message['candidate'],
                    'sdpMLineIndex': message.get('sdpMLineIndex'),
                    'sdpMid': message.get('sdpMid'),
                    'from': client_id,
                    'target': target
                })
            else:
                # Broadcast ICE candidate
                await self.broadcast({
                    'type': 'ice',
                    'candidate': message['candidate'],
                    'sdpMLineIndex': message.get('sdpMLineIndex'),
                    'sdpMid': message.get('sdpMid'),
                    'from': client_id
                }, exclude={client_id})
                
        elif msg_type == 'register':
            # Client registration (can be called multiple times for same WebSocket with different client_ids)
            new_client_id = message.get('client_id')
            if new_client_id:
                # Only broadcast peer-joined if this is a NEW client_id (avoid duplicates)
                is_new = new_client_id not in self.clients
                await self.register_client(websocket, new_client_id)
                await websocket.send(json.dumps({
                    'type': 'registered',
                    'client_id': new_client_id
                }))
                self.logger.info(f'Client registered as {new_client_id}')
                
                # Only notify other clients if this is a genuinely new peer
                if is_new:
                    await self.broadcast({
                        'type': 'peer-joined',
                        'peer_id': new_client_id,
                        'role': message.get('role', 'unknown')
                    }, exclude={new_client_id})
            
        elif msg_type == 'session_rejected':
            # Forward session rejection to the target client
            target = message.get('target')
            if target:
                await self.send_to_client(target, {
                    'type': 'session_rejected',
                    'reason': message.get('reason', 'Session rejected'),
                    'from': client_id
                })
                self.logger.info(f'Forwarded session_rejected to {target}')
            else:
                self.logger.warning('session_rejected message missing target')

        elif msg_type == 'ping':
            # Respond to ping for connection keepalive
            await websocket.send(json.dumps({
                'type': 'pong',
                'timestamp': message.get('timestamp')
            }))
            
        else:
            self.logger.warning(f'Unknown message type: {msg_type}')

    async def handle_client(self, websocket: WebSocketServerProtocol):
        """Handle individual client connection"""
        client_id = None
        # Track ALL client_ids registered by this WebSocket (e.g. operator_recv + operator_send)
        all_client_ids: list = []
        
        try:
            # First message should contain client ID
            first_message = await websocket.recv()
            data = json.loads(first_message)
            
            client_id = data.get('client_id')
            if not client_id:
                self.logger.error('Client did not provide client_id')
                await websocket.close()
                return
                
            await self.register_client(websocket, client_id)
            all_client_ids.append(client_id)

            # Send acknowledgment
            await websocket.send(json.dumps({
                'type': 'registered',
                'client_id': client_id
            }))

            # Notify other clients that a new peer joined (so robot can start pipeline)
            await self.broadcast({
                'type': 'peer-joined',
                'peer_id': client_id,
                'role': data.get('role', 'unknown')
            }, exclude={client_id})

            # Send list of already-connected peers so late joiners (e.g. robot
            # bridge reconnecting after service restart) discover existing clients.
            for existing_id in list(self.clients.keys()):
                if existing_id != client_id:
                    await websocket.send(json.dumps({
                        'type': 'peer-joined',
                        'peer_id': existing_id,
                    }))

            # Handle initial message if it contains signaling data (but skip 'register' - already handled above)
            if data.get('type') and data.get('type') != 'register':
                await self.handle_message(websocket, client_id, data)
            
            # Process subsequent messages
            async for message_str in websocket:
                try:
                    message = json.loads(message_str)
                    # Track additional registrations from this WebSocket
                    if message.get('type') == 'register' and message.get('client_id'):
                        new_id = message['client_id']
                        if new_id not in all_client_ids:
                            all_client_ids.append(new_id)
                    # Use the message's 'from' field when it matches a registered
                    # client_id for this WebSocket (one WS can carry multiple identities,
                    # e.g. operator_recv + operator_send).
                    msg_from = message.get('from')
                    effective_id = msg_from if msg_from in all_client_ids else client_id
                    await self.handle_message(websocket, effective_id, message)
                except json.JSONDecodeError as e:
                    self.logger.error(f'Failed to parse message from {client_id}: {e}')
                except Exception as e:
                    self.logger.error(f'Error handling message from {client_id}: {e}')
                    
        except websockets.exceptions.ConnectionClosed:
            self.logger.info(f'Connection closed for {client_id}')
        except Exception as e:
            self.logger.error(f'Error in handle_client: {e}')
        finally:
            # Unregister ALL client_ids and broadcast peer-left for each
            for cid in all_client_ids:
                try:
                    await self.unregister_client(cid)
                except Exception:
                    pass
                try:
                    await self.broadcast({'type': 'peer-left', 'peer_id': cid})
                except Exception:
                    pass

    async def start(self):
        """Start the signaling server"""
        self.logger.info(f'Starting signaling server on {self.host}:{self.port}')
        
        async with serve(self.handle_client, self.host, self.port):
            self.logger.info(f'Signaling server running on ws://{self.host}:{self.port}')
            await asyncio.Future()  # Run forever


def main():
    """Main entry point for standalone signaling server"""
    import argparse

    parser = argparse.ArgumentParser(description='WebRTC Signaling Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8443, help='Port to bind to')
    args = parser.parse_args()

    # Suppress "Cannot call write() after write_eof()" from websockets during Ctrl+C teardown
    def _exception_handler(loop, ctx):
        exc = ctx.get('exception')
        if exc and 'write_eof' in str(exc):
            return
        loop.default_exception_handler(ctx)

    server = SignalingServer(host=args.host, port=args.port)
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.set_exception_handler(_exception_handler)
        loop.run_until_complete(server.start())
    except KeyboardInterrupt:
        logging.info('Shutting down signaling server')
    finally:
        try:
            loop = asyncio.get_event_loop()
            if not loop.is_closed():
                loop.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
