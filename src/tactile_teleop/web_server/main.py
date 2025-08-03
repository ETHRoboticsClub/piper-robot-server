import argparse
import asyncio
import logging
import signal
from contextlib import asynccontextmanager
from dataclasses import replace

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from tactile_teleop.config import TelegripConfig, config as global_config
from tactile_teleop.utils import get_local_ip, get_web_server_path

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """FastAPI lifespan context manager for startup and shutdown."""
    logger.info("🚀 FastAPI web server starting up")
    yield
    logger.info("🛑 FastAPI web server shutting down")


def create_app(config: TelegripConfig, behind_proxy: bool = False) -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(
        title="Tactile Teleop Web Server",
        description="Static file server for the teleoperation web interface",
        version="1.0.0",
        docs_url=None,  # Disable docs in production
        redoc_url=None,  # Disable redoc in production
        lifespan=lifespan,
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, specify exact origins
        allow_credentials=True,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
    )
    
    if behind_proxy:
        # Add health check endpoint for nginx
        @app.get("/health")
        async def health_check():
            return {"status": "healthy", "timestamp": asyncio.get_event_loop().time()}
        
        # Add API routes under /api prefix when behind proxy
        @app.get("/api/info")
        async def api_info():
            return {
                "app": "Tactile Teleop Web Server",
                "version": "1.0.0",
                "status": "running"
            }
    
    else:
        # serve static files with fastapi
        web_ui_path = get_web_server_path("web-ui") 
        if web_ui_path.exists():
            app.mount("/", StaticFiles(directory=str(web_ui_path), html=True), name="static")
        else:
            logger.error(f"Web UI directory not found: {web_ui_path}")
            raise FileNotFoundError(f"Web UI directory not found: {web_ui_path}")
        

    return app


class FastAPIServer:
    """FastAPI server wrapper for the teleoperation web interface."""

    def __init__(self, config: TelegripConfig, behind_proxy: bool = False, uds_path: str | None = None):
        self.config = config
        self.behind_proxy = behind_proxy
        self.uds_path = uds_path
        self.app = create_app(config, behind_proxy)
        self.server = None
        self.system_ref = None  # For future API endpoints

    def set_system_ref(self, system_ref):
        """Set reference to the main teleoperation system for future API endpoints."""
        self.system_ref = system_ref

    async def start(self):
        """Start the FastAPI server with uvicorn."""
        try:
            if self.uds_path:
                # Unix domain socket configuration (for nginx proxy)
                uvicorn_config = uvicorn.Config(
                    app=self.app,
                    uds=self.uds_path, # listens to unix domain socket
                    log_level=self.config.log_level.lower(),
                    access_log=getattr(logging, self.config.log_level.upper()) <= logging.INFO,
                    use_colors=False,  # Disable colors for consistent logging
                )
                
                # Only log startup info if INFO level or more verbose
                if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                    logger.info(f"FastAPI server started on Unix socket: {self.uds_path}")
                    
            else:
                # TCP socket configuration (direct access to web server)
                ssl_keyfile = None
                ssl_certfile = None
                
                if self.config.ssl_files_exist:
                    ssl_certfile, ssl_keyfile = self.config.get_absolute_ssl_paths()
                else:
                    logger.warning("SSL certificates not found. Starting HTTP server on port 8080")
                    # Fallback to HTTP for development
                    self.config = replace(self.config, https_port=8080)

                uvicorn_config = uvicorn.Config(
                    app=self.app,
                    host=self.config.host_ip,
                    port=self.config.https_port,
                    ssl_keyfile=ssl_keyfile,
                    ssl_certfile=ssl_certfile,
                    log_level=self.config.log_level.lower(),
                    access_log=getattr(logging, self.config.log_level.upper()) <= logging.INFO,
                    use_colors=False,  # Disable colors for consistent logging
                )

                # Only log startup info if INFO level or more verbose
                if getattr(logging, self.config.log_level.upper()) <= logging.INFO:
                    protocol = "HTTPS" if ssl_certfile else "HTTP"
                    logger.info(f"{protocol} server started on {self.config.host_ip}:{self.config.https_port}")

            self.server = uvicorn.Server(uvicorn_config)

        except Exception as e:
            logger.error(f"Failed to start FastAPI server: {e}")
            raise

    async def serve(self):
        """Start serving the FastAPI application."""
        if not self.server:
            await self.start()
        await self.server.serve()

    async def stop(self):
        """Stop the FastAPI server."""
        if self.server:
            self.server.should_exit = True
            logger.info("FastAPI server stopped")


async def _serve_forever(server: FastAPIServer):
    """Keep the FastAPI server alive until an exit signal arrives."""
    # Setup graceful shutdown
    stop_event = asyncio.Event()

    def _graceful_shutdown(*_):
        logger.info("Received shutdown signal")
        stop_event.set()

    # Handle Ctrl-C or kill signals
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _graceful_shutdown)

    # Start server and wait for shutdown signal
    server_task = asyncio.create_task(server.serve())
    stop_task = asyncio.create_task(stop_event.wait())

    try:
        # Wait for either server completion or stop signal
        done, pending = await asyncio.wait(
            [server_task, stop_task], return_when=asyncio.FIRST_COMPLETED
        )

        # Cancel remaining tasks
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        await server.stop()


async def main() -> None:
    """Main function for the FastAPI web server."""
    parser = argparse.ArgumentParser(description="Tactile Teleop FastAPI Web Server")
    parser.add_argument(
        "--host", 
        default=global_config.host_ip, 
        help="IP/interface to bind (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", 
        type=int, 
        default=global_config.https_port, 
        help="HTTPS port (default: 8443)"
    )
    parser.add_argument(
        "--log-level",
        default=global_config.log_level,
        choices=["debug", "info", "warning", "error", "critical"],
        help="Logging verbosity",
    )
    parser.add_argument(
        "--behind-proxy",
        action="store_true",
        help="Run behind nginx proxy (disables static file serving)",
    )
    parser.add_argument(
        "--uds",
        help="Unix domain socket path (default: /tmp/tactile-teleop.sock)",
    )
    args = parser.parse_args()

    # Configure root logger *before* any further logging
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s - %(message)s",
        datefmt="%H:%M:%S",
    )

    # Build config with CLI overrides
    cfg: TelegripConfig = replace(
        global_config, 
        host_ip=args.host, 
        https_port=args.port, 
        log_level=args.log_level
    )

    socket_path = None
    if args.behind_proxy:
        logger.info("🖥️  Tactile Teleop FastAPI web server starting (nginx proxy mode)")
        # Auto-generate socket path (behind proxy, without --uds flag)
        socket_path = args.uds if args.uds else "/tmp/tactile-teleop.sock"
        logger.info("🔌 Unix socket: %s", socket_path)
        logger.info("🌐 Nginx handles SSL and serves static files")
        logger.info("💡 Use --log-level info to see detailed output\n")
        logger.info("💡 Use --uds to customize the socket path\n")
    else:
        # TCP socket mode - direct access
        # Ensure SSL certificates exist for HTTPS
        if not cfg.ssl_files_exist:
            logger.info("SSL certificates not found, attempting to generate...")
            cfg.ensure_ssl_certificates()

        host_display = get_local_ip() if cfg.host_ip == "0.0.0.0" else cfg.host_ip
        protocol = "HTTPS" if cfg.ssl_files_exist else "HTTP"
        
        logger.info("🖥️  Tactile Teleop FastAPI web server starting on %s:%s", host_display, cfg.https_port)
        logger.info("📱 Open the UI in your browser:")
        logger.info("   %s://%s:%s", protocol.lower(), host_display, cfg.https_port)
        logger.info("📱 Then go to the same address on your VR headset browser")
        logger.info("💡 Use --log-level info to see detailed output\n")

    await _serve_forever(FastAPIServer(cfg, behind_proxy=args.behind_proxy, uds_path=socket_path))
    logger.info("✅ Web server shutdown complete.")


def main_cli() -> None:
    """Sync wrapper so the file is runnable as a script or module."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("👋 Web server interrupted")


if __name__ == "__main__":
    main_cli()