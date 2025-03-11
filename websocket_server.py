#!/usr/bin/env python3
"""
WebSocket Server for Autonomous Vehicle Telemetry

This script runs a WebSocket server that receives telemetry data from the autonomous vehicle
and serves it to web clients for display.

Usage:
    python websocket_server.py [--host HOST] [--port PORT]

Options:
    --host HOST    Hostname to bind the server to (default: 0.0.0.0)
    --port PORT    Port to listen on (default: 8090)
"""

import asyncio
import json
import logging
import argparse
import websockets
import signal
import time
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('websocket_server')

# Global variables
connected_clients = set()
latest_telemetry = {}
start_time = time.time()

async def telemetry_handler(websocket, path):
    """Handle WebSocket connection for telemetry data"""
    global connected_clients, latest_telemetry
    
    # Determine client type from path
    if path == "/telemetry":
        # This is the vehicle sending telemetry
        logger.info(f"Vehicle connected from {websocket.remote_address}")
        
        try:
            async for message in websocket:
                try:
                    # Parse the telemetry data
                    data = json.loads(message)
                    
                    # Add timestamp
                    data['timestamp'] = datetime.now().isoformat()
                    data['uptime'] = time.time() - start_time
                    
                    # Store latest telemetry
                    latest_telemetry = data
                    
                    # Broadcast to all viewer clients
                    if connected_clients:
                        logger.debug(f"Broadcasting telemetry to {len(connected_clients)} clients")
                        await asyncio.gather(
                            *[client.send(json.dumps(data)) for client in connected_clients]
                        )
                    
                except json.JSONDecodeError:
                    logger.error("Received invalid JSON data")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Vehicle disconnected from {websocket.remote_address}")
        
    elif path == "/viewer":
        # This is a viewer client
        logger.info(f"Viewer client connected from {websocket.remote_address}")
        connected_clients.add(websocket)
        
        try:
            # Send the latest telemetry data immediately
            if latest_telemetry:
                await websocket.send(json.dumps(latest_telemetry))
            
            # Keep the connection open
            await websocket.wait_closed()
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Viewer client disconnected from {websocket.remote_address}")
        finally:
            connected_clients.remove(websocket)

async def start_server(host, port):
    """Start the WebSocket server"""
    server = await websockets.serve(
        telemetry_handler,
        host,
        port,
        ping_interval=30,
        ping_timeout=10
    )
    
    logger.info(f"WebSocket server started on {host}:{port}")
    
    # Setup graceful shutdown
    loop = asyncio.get_running_loop()
    for signal_name in ('SIGINT', 'SIGTERM'):
        loop.add_signal_handler(
            getattr(signal, signal_name),
            lambda: asyncio.create_task(shutdown(server))
        )
    
    await server.wait_closed()

async def shutdown(server):
    """Gracefully shut down the server"""
    logger.info("Shutting down server...")
    server.close()
    await server.wait_closed()
    
    # Close all client connections
    if connected_clients:
        logger.info(f"Closing {len(connected_clients)} client connections")
        for websocket in connected_clients:
            await websocket.close()
    
    # Stop the event loop
    asyncio.get_event_loop().stop()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="WebSocket Server for Autonomous Vehicle Telemetry")
    parser.add_argument("--host", default="0.0.0.0", help="Hostname to bind the server to")
    parser.add_argument("--port", type=int, default=8090, help="Port to listen on")
    args = parser.parse_args()
    
    logger.info(f"Starting WebSocket server on {args.host}:{args.port}")
    
    try:
        asyncio.run(start_server(args.host, args.port))
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server error: {e}")

if __name__ == "__main__":
    main()