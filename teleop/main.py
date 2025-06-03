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


class HTTPSServer:
    """HTTPS server for serving the web application."""
    
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.httpd = None
        self.server_task = None
    
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
            
            # Create server
            handler = http.server.SimpleHTTPRequestHandler
            self.httpd = http.server.HTTPServer((self.config.host_ip, self.config.https_port), handler)
            
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
            else:
                logger.info(f"Access webapp at: https://localhost:{self.config.https_port}/")
            
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
        
        # Components
        self.https_server = HTTPSServer(config)
        self.vr_server = VRWebSocketServer(self.command_queue, config)
        self.keyboard_listener = KeyboardListener(self.command_queue, config)
        self.control_loop = ControlLoop(self.command_queue, config)
        
        # Tasks
        self.tasks = []
        self.is_running = False
    
    async def start(self):
        """Start all system components."""
        logger.info("Starting Unified Teleoperation System...")
        logger.info("="*60)
        
        self.is_running = True
        
        # Start HTTPS server
        if not await self.https_server.start():
            logger.warning("HTTPS server failed to start")
        
        # Start input providers
        await self.vr_server.start()
        await self.keyboard_listener.start()
        
        # Start control loop
        control_task = asyncio.create_task(self.control_loop.start())
        self.tasks.append(control_task)
        
        logger.info("="*60)
        logger.info("🎉 Teleoperation system ready!")
        logger.info("VR Controllers: Connect your Quest and use grip buttons to control arms")
        logger.info("Keyboard: Press Enter to activate, use WASD+QE for movement, 1/2 to switch arms")
        logger.info("Web Interface: Check the HTTPS server address above")
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
    parser.add_argument("--left-port", default="/dev/ttySO100follower", help="Left arm serial port")
    parser.add_argument("--right-port", default="/dev/ttySO100leader", help="Right arm serial port")
    
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