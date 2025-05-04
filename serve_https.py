import http.server
import ssl
import os
import socket

PORT = 8443
HOST_IP = '0.0.0.0'
CERTFILE = 'cert.pem'
KEYFILE = 'key.pem'

# Check if certificate and key files exist
if not os.path.exists(CERTFILE) or not os.path.exists(KEYFILE):
    print(f"Error: Certificate ('{CERTFILE}') or Key ('{KEYFILE}') not found.")
    print("Please generate them first, e.g., using openssl:")
    print("openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -subj \"/C=US/ST=Test/L=Test/O=Test/OU=Test/CN=localhost\"")
    exit(1)

# Basic HTTP request handler
handler = http.server.SimpleHTTPRequestHandler

# Create SSL context and wrap the server socket
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile=CERTFILE, keyfile=KEYFILE)

httpd = http.server.HTTPServer((HOST_IP, PORT), handler)
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

# --- Get and print accessible IP ---
def get_lan_ip():
    """Attempts to find the LAN IP address."""
    s = None
    try:
        # Connect to an external host (doesn't send data)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.1) # Prevent hanging if no route
        # Use a common public DNS server or a non-routable address
        # Using 10.255.255.255 can work even offline on some systems
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
        return IP
    except Exception:
        # Fallback: Try getting IP associated with hostname
        try:
            return socket.gethostbyname(socket.gethostname())
        except socket.gaierror:
            return None # Could not determine IP
    finally:
        if s:
            s.close()

lan_ip = get_lan_ip()

if lan_ip and not lan_ip.startswith('127.'):
    print(f"Serving HTTPS on {lan_ip} port {PORT} (https://{lan_ip}:{PORT}/)...")
    print(f"(Also listening on all interfaces: {HOST_IP}:{PORT})")
else:
    print(f"Serving HTTPS on all interfaces ({HOST_IP}) port {PORT}...")
    print("Could not automatically determine the primary network IP.")
    print(f"Access the server via https://<your-machine-ip>:{PORT}/ or https://localhost:{PORT}/ (if on the same machine).")
# --- End IP printing ---

print("Use Ctrl+C to stop.")

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    print("\nServer stopped.")
    httpd.server_close() 