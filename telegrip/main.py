"""
The main entry point for the teleoperation system.
"""

import argparse
import asyncio
import http.server
import logging
import ssl
import threading

from .config import TelegripConfig, config
from .control_loop import ControlLoop
from .inputs.vr_controllers import VRControllerInputProvider
from .utils import get_local_ip

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


async def main():
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")

    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--vis", action="store_true", help="Enable visualization")
    parser.add_argument(
        "--log-level",
        default="info",
        choices=["debug", "info", "warning", "error", "critical"],
        help="Set logging level",
    )

    args = parser.parse_args()

    # Configure logging
    log_level = getattr(logging, args.log_level.upper())
    logging.basicConfig(level=log_level, format="%(asctime)s - %(message)s", datefmt="%H:%M:%S")

    robot_enabled = not args.no_robot
    visualize = args.vis
    host_display = get_local_ip() if config.host_ip == "0.0.0.0" else config.host_ip
    logging.info("🤖 telegrip starting...")
    logging.info("📱 Open the UI in your browser on:")
    logging.info(f"   https://{host_display}:{config.https_port}")
    logging.info("📱 Then go to the same address on your VR headset browser")
    logging.info("💡 Use --log-level info to see detailed output")
    logging.info("")

    try:
        command_queue = asyncio.Queue()
        https_server = HTTPSServer(config)
        vr_control_server = VRControllerInputProvider(command_queue, config)
        control_loop = ControlLoop(config, robot_enabled, visualize)

        await https_server.start()
        vr_control_task = asyncio.create_task(vr_control_server.start('robot-vr-teleop-room', 'vr-teleop-viewer'))
        control_loop_task = asyncio.create_task(control_loop.run(command_queue))
        await asyncio.gather(control_loop_task, vr_control_task)

    except KeyboardInterrupt:
        logging.info("\n🛑 Keyboard interrupt. Shutting down...")
    except asyncio.CancelledError:
        logging.info("\n🛑 Task cancelled. Shutting down...")
    except Exception as e:
        logging.error(f"🚨 Unexpected error: {e}")
    finally:
        try:
            control_loop_task.cancel()
            try:
                await asyncio.wait_for(asyncio.gather(control_loop_task, return_exceptions=True), timeout=5.0)
            except asyncio.TimeoutError:
                logger.warning("Some tasks did not complete within timeout")
            except KeyboardInterrupt:
                logger.warning("Cleanup interrupted, forcing shutdown")

            await https_server.stop()
            await control_loop.stop()
            logging.info("✅ Shutdown complete.")

        except KeyboardInterrupt:
            logger.warning("Cleanup forcibly interrupted")
        except Exception as e:
            logger.error(f"Error during cleanup: {e}")


def main_cli():
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("👋 telegrip stopped")


if __name__ == "__main__":
    main_cli()
