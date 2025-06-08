"""
Main entry point for the unified teleoperation system.
Coordinates HTTPS server, WebSocket server, robot interface, and input providers.
"""

import asyncio
import argparse
import logging
import signal
import sys
import http.server
import ssl
import socket
import json
import urllib.parse
import time
from typing import Optional

from .config import TeleopConfig
from .control_loop import ControlLoop
from .inputs.vr_ws_server import VRWebSocketServer
from .inputs.keyboard_listener import KeyboardListener
from .inputs.base import ControlGoal

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class APIRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP request handler with API support."""
    
    def __init__(self, *args, system_ref=None, **kwargs):
        self.system_ref = system_ref
        super().__init__(*args, **kwargs)
    
    def do_GET(self):
        """Handle GET requests."""
        parsed_path = urllib.parse.urlparse(self.path)
        
        if parsed_path.path == '/api/status':
            self.handle_status_request()
        else:
            # Serve static files
            super().do_GET()
    
    def do_POST(self):
        """Handle POST requests."""
        parsed_path = urllib.parse.urlparse(self.path)
        
        if parsed_path.path == '/api/control':
            self.handle_control_request()
        else:
            self.send_error(404, "API endpoint not found")
    
    def handle_status_request(self):
        """Handle system status requests."""
        try:
            status = self.get_system_status()
            self.send_json_response(status)
        except Exception as e:
            logger.error(f"Error handling status request: {e}")
            self.send_error(500, "Internal server error")
    
    def handle_control_request(self):
        """Handle control requests (keyboard toggle, etc.)."""
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self.send_error(400, "No request body")
                return
            
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            result = self.process_control_command(data)
            self.send_json_response(result)
            
        except json.JSONDecodeError:
            self.send_error(400, "Invalid JSON")
        except Exception as e:
            logger.error(f"Error handling control request: {e}")
            self.send_error(500, "Internal server error")
    
    def get_system_status(self):
        """Get current system status."""
        if not self.system_ref:
            return {
                "leftArm": False,
                "rightArm": False,
                "vrConnected": False,
                "keyboardEnabled": False,
                "error": "System reference not available"
            }
        
        system = self.system_ref()
        if not system:
            return {
                "leftArm": False,
                "rightArm": False,
                "vrConnected": False,
                "keyboardEnabled": False,
                "error": "System not available"
            }
        
        # Get robot status
        robot_status = {"leftArm": False, "rightArm": False}
        if system.control_loop and system.control_loop.robot_interface:
            robot_interface = system.control_loop.robot_interface
            # Use individual arm connection status instead of overall connection
            robot_status["leftArm"] = robot_interface.get_arm_connection_status("left")
            robot_status["rightArm"] = robot_interface.get_arm_connection_status("right")
        
        # Get VR status
        vr_connected = False
        if system.vr_server and hasattr(system.vr_server, 'clients'):
            vr_connected = len(system.vr_server.clients) > 0
        
        # Get keyboard status
        keyboard_enabled = False
        if system.keyboard_listener:
            keyboard_enabled = system.keyboard_listener.is_running
        
        return {
            "leftArm": robot_status["leftArm"],
            "rightArm": robot_status["rightArm"],
            "vrConnected": vr_connected,
            "keyboardEnabled": keyboard_enabled,
            "timestamp": time.time()
        }
    
    def process_control_command(self, data):
        """Process control commands."""
        action = data.get('action')
        
        if action == 'enable_keyboard':
            return self.toggle_keyboard(True)
        elif action == 'disable_keyboard':
            return self.toggle_keyboard(False)
        elif action == 'get_status':
            return self.get_system_status()
        else:
            return {"success": False, "error": f"Unknown action: {action}"}
    
    def toggle_keyboard(self, enabled):
        """Toggle keyboard control."""
        if not self.system_ref:
            return {"success": False, "error": "System reference not available"}
        
        system = self.system_ref()
        if not system or not system.keyboard_listener:
            return {"success": False, "error": "Keyboard listener not available"}
        
        try:
            # Put command in queue for async processing
            action = 'enable_keyboard' if enabled else 'disable_keyboard'
            command = {'action': action}
            
            # Use put_nowait since we're in a sync context
            try:
                system.control_commands_queue.put_nowait(command)
            except asyncio.QueueFull:
                return {"success": False, "error": "Control commands queue is full"}
            
            return {
                "success": True, 
                "enabled": enabled,
                "message": f"Keyboard control {'enable' if enabled else 'disable'} requested"
            }
        except Exception as e:
            logger.error(f"Error toggling keyboard: {e}")
            return {"success": False, "error": str(e)}
    
    def send_json_response(self, data):
        """Send JSON response."""
        response = json.dumps(data).encode('utf-8')
        
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(response))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        
        self.wfile.write(response)
    
    def do_OPTIONS(self):
        """Handle CORS preflight requests."""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()


class HTTPSServer:
    """HTTPS server for serving the web application."""
    
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.httpd = None
        self.server_task = None
        self.system_ref = None  # Weak reference to main system
    
    def set_system_ref(self, system_ref):
        """Set reference to main system for API calls."""
        self.system_ref = system_ref
    
    def get_lan_ip(self) -> Optional[str]:
        """Get the LAN IP address."""
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.1)
            s.connect(('10.255.255.255', 1))
            return s.getsockname()[0]
        except Exception:
            try:
                return socket.gethostbyname(socket.gethostname())
            except socket.gaierror:
                return None
        finally:
            if s:
                s.close()
    
    async def start(self):
        """Start the HTTPS server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates")
                logger.error("Manual generation may be required:")
                logger.error("openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -subj \"/C=US/ST=Test/L=Test/O=Test/OU=Test/CN=localhost\"")
                return False
        
        if not self.config.webapp_exists:
            logger.error(f"Webapp directory '{self.config.webapp_dir}' not found")
            return False
        
        try:
            # Change to webapp directory for serving files
            import os
            original_cwd = os.getcwd()
            os.chdir(self.config.webapp_dir)
            
            # Create handler class with system reference
            def handler_factory(*args, **kwargs):
                return APIRequestHandler(*args, system_ref=self.system_ref, **kwargs)
            
            # Create server
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), handler_factory)
            
            # Setup SSL
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(certfile=f"../{self.config.certfile}", keyfile=f"../{self.config.keyfile}")
            self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)
            
            # Start server in background
            loop = asyncio.get_event_loop()
            self.server_task = loop.run_in_executor(None, self.httpd.serve_forever)
            
            # Log access information
            lan_ip = self.get_lan_ip()
            logger.info(f"HTTPS server started on port {self.config.https_port}")
            if lan_ip and not lan_ip.startswith('127.'):
                logger.info(f"Access webapp at: https://{lan_ip}:{self.config.https_port}/")
                logger.info(f"API available at: https://{lan_ip}:{self.config.https_port}/api/")
            else:
                logger.info(f"Access webapp at: https://localhost:{self.config.https_port}/")
                logger.info(f"API available at: https://localhost:{self.config.https_port}/api/")
            
            # Restore original directory
            os.chdir(original_cwd)
            return True
            
        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
            return False
    
    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_task:
                self.server_task.cancel()
            logger.info("HTTPS server stopped")


class TeleopSystem:
    """Main teleoperation system coordinator."""
    
    def __init__(self, config: TeleopConfig):
        self.config = config
        
        # Command queue for communication between input providers and control loop
        self.command_queue = asyncio.Queue(maxsize=100)
        
        # Control commands queue for web interface
        self.control_commands_queue = asyncio.Queue(maxsize=10)
        
        # Components
        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.keyboard_listener = KeyboardListener(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config)
        
        # Set system reference for API calls
        import weakref
        self.https_server.set_system_ref(weakref.ref(self))
        
        # Tasks
        self.tasks = []
        self.is_running = False
    
    async def process_control_commands(self):
        """Process control commands from the web interface."""
        while self.is_running:
            try:
                command = await asyncio.wait_for(self.control_commands_queue.get(), timeout=1.0)
                action = command.get('action')
                
                if action == 'enable_keyboard':
                    if not self.keyboard_listener.is_running:
                        await self.keyboard_listener.start()
                        logger.info("Keyboard control enabled via web interface")
                elif action == 'disable_keyboard':
                    if self.keyboard_listener.is_running:
                        await self.keyboard_listener.stop()
                        logger.info("Keyboard control disabled via web interface")
                        
                # Mark command as done
                self.control_commands_queue.task_done()
                
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"Error processing control command: {e}")
    
    async def start(self):
        """Start all system components."""
        logger.info("Starting Unified Teleoperation System...")
        logger.info("="*60)
        
        self.is_running = True
        
        # Start HTTPS server
        if not await self.https_server.start():
            logger.warning("HTTPS server failed to start")
        
        # Start control loop first (includes robot setup)
        control_task = asyncio.create_task(self.control_loop.start())
        self.tasks.append(control_task)
        
        # Start control commands processor
        control_commands_task = asyncio.create_task(self.process_control_commands())
        self.tasks.append(control_commands_task)
        
        # Wait a moment for control loop to setup
        await asyncio.sleep(0.5)
        
        # Connect keyboard listener to robot interface after control loop setup
        if self.control_loop.robot_interface:
            self.keyboard_listener.set_robot_interface(self.control_loop.robot_interface)
            logger.info("Connected keyboard listener to robot interface")
        else:
            logger.warning("Robot interface not available for keyboard listener")
        
        # Start input providers
        await self.vr_server.start()
        # Note: keyboard listener will be started on-demand via web interface
        
        logger.info("="*60)
        logger.info("🎉 Teleoperation system ready!")
        logger.info("VR Controllers: Connect your Quest and use grip buttons to control arms")
        logger.info("Web Interface: Use the web interface to enable keyboard control")
        logger.info("Press Ctrl+C to shutdown")
        logger.info("="*60)
    
    async def stop(self):
        """Stop all system components."""
        logger.info("Shutting down teleoperation system...")
        self.is_running = False
        
        # Stop input providers
        await self.vr_server.stop()
        await self.keyboard_listener.stop()
        
        # Stop control loop
        await self.control_loop.stop()
        
        # Stop HTTPS server
        await self.https_server.stop()
        
        # Cancel all tasks
        for task in self.tasks:
            if not task.done():
                task.cancel()
        
        # Wait for tasks to complete
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
        
        logger.info("Teleoperation system shutdown complete")
    
    async def run(self):
        """Run the teleoperation system."""
        try:
            await self.start()
            
            # Run until interrupted
            while self.is_running:
                await asyncio.sleep(1)
                
        except KeyboardInterrupt:
            logger.info("Ctrl+C detected, shutting down...")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
        finally:
            await self.stop()


def setup_signal_handlers():
    """Setup signal handlers for graceful shutdown."""
    def signal_handler(sig, frame):
        logger.info(f"Received signal {sig}")
        # The main loop will catch KeyboardInterrupt
        raise KeyboardInterrupt()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Unified SO100 Robot Teleoperation System")
    
    # Control flags
    parser.add_argument("--no-robot", action="store_true", help="Disable robot connection (visualization only)")
    parser.add_argument("--no-viz", action="store_true", help="Disable PyBullet visualization")
    parser.add_argument("--no-vr", action="store_true", help="Disable VR WebSocket server")
    parser.add_argument("--no-keyboard", action="store_true", help="Disable keyboard input")
    parser.add_argument("--no-https", action="store_true", help="Disable HTTPS server")
    
    # Network settings
    parser.add_argument("--https-port", type=int, default=8443, help="HTTPS server port")
    parser.add_argument("--ws-port", type=int, default=8442, help="WebSocket server port")
    parser.add_argument("--host", default="0.0.0.0", help="Host IP address")
    
    # Paths
    parser.add_argument("--urdf", default="URDF/SO_5DOF_ARM100_8j/urdf/so100.urdf", help="Path to robot URDF file")
    parser.add_argument("--webapp", default="webapp", help="Path to webapp directory")
    parser.add_argument("--cert", default="cert.pem", help="Path to SSL certificate")
    parser.add_argument("--key", default="key.pem", help="Path to SSL private key")
    
    # Robot settings
    parser.add_argument("--left-port", default="/dev/ttySO100red", help="Left arm serial port")
    parser.add_argument("--right-port", default="/dev/ttySO100blue", help="Right arm serial port")
    
    return parser.parse_args()


def create_config_from_args(args) -> TeleopConfig:
    """Create configuration from command line arguments."""
    config = TeleopConfig()
    
    # Apply command line overrides
    config.enable_robot = not args.no_robot
    config.enable_pybullet = not args.no_viz
    config.enable_vr = not args.no_vr
    config.enable_keyboard = not args.no_keyboard
    
    config.https_port = args.https_port
    config.websocket_port = args.ws_port
    config.host_ip = args.host
    
    config.urdf_path = args.urdf
    config.webapp_dir = args.webapp
    config.certfile = args.cert
    config.keyfile = args.key
    
    config.follower_ports = {
        "left": args.left_port,
        "right": args.right_port
    }
    
    return config


async def main():
    """Main entry point."""
    # Setup signal handlers
    setup_signal_handlers()
    
    # Parse arguments
    args = parse_arguments()
    config = create_config_from_args(args)
    
    # Log configuration
    logger.info("Starting with configuration:")
    logger.info(f"  Robot: {'enabled' if config.enable_robot else 'disabled'}")
    logger.info(f"  PyBullet: {'enabled' if config.enable_pybullet else 'disabled'}")
    logger.info(f"  VR: {'enabled' if config.enable_vr else 'disabled'}")
    logger.info(f"  Keyboard: {'enabled' if config.enable_keyboard else 'disabled'}")
    logger.info(f"  HTTPS Port: {config.https_port}")
    logger.info(f"  WebSocket Port: {config.websocket_port}")
    logger.info(f"  Robot Ports: {config.follower_ports}")
    
    # Create and run system
    system = TeleopSystem(config)
    await system.run()


def main_cli():
    """Console script entry point for pip-installed package."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown complete.")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main_cli() 