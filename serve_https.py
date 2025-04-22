import http.server
import ssl
import os

PORT = 8443
HOST_IP = '192.168.7.136'
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

print(f"Serving HTTPS on {HOST_IP} port {PORT} (https://{HOST_IP}:{PORT}/)...")
print("Use Ctrl+C to stop.")

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    print("\nServer stopped.")
    httpd.server_close() 