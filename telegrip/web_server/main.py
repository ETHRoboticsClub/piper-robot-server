import argparse
import http.server
import ssl
import threading
import logging
import asyncio
import signal
from dataclasses import replace

from telegrip.utils import get_local_ip
from telegrip.config import config as global_config

from telegrip.config import TelegripConfig

logger = logging.getLogger(__name__)


class APIHandler(http.server.BaseHTTPRequestHandler):
    """HTTP request handler for the teleoperation API."""

    def __init__(self, *args, **kwargs):
        # Set CORS headers for all requests
        super().__init__(*args, **kwargs)

    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        try:
            super().end_headers()
        except (
            BrokenPipeError,
            ConnectionResetError,
            ConnectionAbortedError,
            ssl.SSLError,
        ):
            # Client disconnected or SSL error - ignore silently
            pass

    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()

    def log_message(self, format, *args):
        """Override to reduce HTTP request logging noise."""
        pass  # Disable default HTTP logging

    def do_GET(self):
        """Handle GET requests."""
        if self.path == "/" or self.path == "/index.html":
            # Serve main page from web-ui directory
            self.serve_file("web-ui/index.html", "text/html")
        elif self.path.endswith(".css"):
            # Serve CSS files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "text/css")
        elif self.path.endswith(".js"):
            # Serve JS files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "application/javascript")
        elif self.path.endswith(".ico"):
            self.serve_file(self.path[1:], "image/x-icon")
        elif self.path.endswith((".jpg", ".jpeg")):
            # Serve image files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "image/jpeg")
        elif self.path.endswith(".png"):
            # Serve image files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "image/png")
        elif self.path.endswith(".gif"):
            # Serve image files from web-ui directory
            self.serve_file(f"web-ui{self.path}", "image/gif")
        else:
            self.send_error(404, "Not found")

    def serve_file(self, filename, content_type):
        """Serve a static file from the project directory."""
        from telegrip.utils import get_absolute_path

        try:
            # Convert relative path to absolute path in project directory
            abs_path = get_absolute_path(filename)

            with open(abs_path, "rb") as f:
                file_content = f.read()

            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", len(file_content))
            self.end_headers()
            self.wfile.write(file_content)

        except FileNotFoundError:
            self.send_error(404, f"File {filename} not found")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            # Client disconnected - log quietly and continue
            logger.debug(f"Client disconnected while serving {filename}")
        except Exception as e:
            logger.error(f"Error serving file {filename}: {e}")
            try:
                self.send_error(500, "Internal server error")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                # Client already disconnected, ignore
                pass


class HTTPSServer:
    """HTTPS server for the teleoperation API."""

    def __init__(self, config: TelegripConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None
        self.system_ref = None  # Direct reference to the main system

    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system."""
        self.system_ref = system_ref

    async def start(self):
        """Start the HTTPS server."""
        try:
            # Create server - directly use APIHandler class
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), APIHandler)

            # Set API handler reference for command queuing
            self.httpd.api_handler = self.system_ref

            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            # Get absolute paths for SSL certificates
            cert_path, key_path = self.config.get_absolute_ssl_paths()
            context.load_cert_chain(cert_path, key_path)
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)

            # Start server in a separate thread
            self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.server_thread.start()

            # Only log if INFO level or more verbose
            if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                logger.info(f"HTTPS server started on {self.config.host_ip}:" f"{self.config.https_port}")

        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            raise

    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            logger.info("HTTPS server stopped")
            
async def _serve_forever(server: HTTPSServer):
    """Keep the HTTPS server alive until an exit signal arrives."""
    await server.start()
    # Wait on an event rather than sleeping in a loop so we can cancel cleanly
    stop = asyncio.Event()

    def _graceful_shutdown(*_):
        stop.set()
    # Handle Ctrl-C or `kill -TERM`
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _graceful_shutdown)

    await stop.wait() # keeps waiting (running the server) until "stop" value is set to true
    await server.stop()

async def main() -> None:
    """Main function for the web-server."""

    parser = argparse.ArgumentParser(description="telegrip Web-UI static file server")
    parser.add_argument("--host", default=global_config.host_ip, help="IP/interface to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=global_config.https_port, help="HTTPS port (default: 8443)")
    parser.add_argument(
        "--log-level",
        default=global_config.log_level,
        choices=["debug", "info", "warning", "error", "critical"],
        help="Logging verbosity",
    )
    args = parser.parse_args()

    # Configure root logger *before* any further logging
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s - %(message)s",
        datefmt="%H:%M:%S",
    )

    # Build a config instance with the overrides supplied on the CLI
    cfg: TelegripConfig = replace(
        global_config, host_ip=args.host, https_port=args.port, log_level=args.log_level
    )

    host_display = get_local_ip() if cfg.host_ip == "0.0.0.0" else cfg.host_ip

    logger.info("🖥️  telegrip web-server starting on %s:%s", host_display, cfg.https_port)
    logger.info("📱 Open the UI in your browser on:")
    logger.info("   https://%s:%s", host_display, cfg.https_port)
    logger.info("📱 Then go to the same address on your VR headset browser")
    logger.info("💡 Use --log-level info to see detailed output\n")

    await _serve_forever(HTTPSServer(cfg))
    logger.info("✅ web-server shutdown complete.")

def main_cli() -> None:
    """Sync wrapper so the file is runnable as a script or module."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("👋 web-server interrupted")

if __name__ == "__main__":
    main_cli()