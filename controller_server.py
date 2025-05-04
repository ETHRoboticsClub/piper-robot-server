import asyncio
import websockets
import json
import socket
import ssl
import os
import logging

# --- Enable Debug Logging for websockets ---
logger = logging.getLogger('websockets')
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())
# --- End Debug Logging Setup ---

# --- SSL Configuration ---
CERTFILE = 'cert.pem'
KEYFILE = 'key.pem'

# Check if certificate and key files exist
if not os.path.exists(CERTFILE) or not os.path.exists(KEYFILE):
    print(f"Error: Certificate ('{CERTFILE}') or Key ('{KEYFILE}') not found for WebSocket server.")
    print("Please ensure these files exist in the same directory.")
    print("You might need to generate them first (see serve_https.py for example command).")
    exit(1)

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
try:
    ssl_context.load_cert_chain(certfile=CERTFILE, keyfile=KEYFILE)
    print("SSL certificate and key loaded successfully for WebSocket server.")
except ssl.SSLError as e:
    print(f"Error loading SSL cert/key: {e}")
    print("Ensure the cert.pem and key.pem files are valid.")
    exit(1)
# --- End SSL Configuration ---

async def handler(websocket, path=None):
    """Handles incoming WebSocket connections and messages."""
    client_address = websocket.remote_address
    print(f"Client connected from {client_address}")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                # Process the received controller data (e.g., print it)
                print(f"Received data: {data}")
                # In a real application, you would use this data
                # (e.g., to control a robot, update a simulation, etc.)

            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
            except Exception as e:
                print(f"Error processing message: {e}")

    except websockets.exceptions.ConnectionClosedOK:
        print(f"Client {client_address} disconnected normally.")
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Client {client_address} disconnected with error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred with client {client_address}: {e}")
    finally:
        print(f"Connection closed for {client_address}")


def get_local_ip():
    """Gets the local IP address of the machine."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't actually establish a connection
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1' # Fallback to localhost
    finally:
        s.close()
    return IP

async def main():
    """Starts the WebSocket server."""
    host_ip = get_local_ip()
    port = 8442 # Standard WebSocket port, change if needed
    print(f"Starting WebSocket server on wss://{host_ip}:{port}")

    async with websockets.serve(handler, host_ip, port, ssl=ssl_context):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    print("Starting server...")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped manually.")
    except Exception as e:
        print(f"Server failed to start or run: {e}") 