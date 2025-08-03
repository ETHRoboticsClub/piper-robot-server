#!/usr/bin/env python3
"""
Docker entrypoint script for Tactile Teleop FastAPI server.
Forces HTTP TCP mode even when behind proxy (for container networking).
"""

import sys
sys.path.insert(0, '/app/src')

from tactile_teleop.web_server.main import FastAPIServer
from tactile_teleop.config import config as global_config
from dataclasses import replace
import asyncio
import logging

async def main():
    """Start FastAPI server in Docker container mode."""
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(message)s",
        datefmt="%H:%M:%S",
    )
    
    logger = logging.getLogger(__name__)
    
    # Build config for Docker container - force HTTP mode
    cfg = replace(
        global_config,
        host_ip="0.0.0.0",
        https_port=8000,  # HTTP port for container networking
        log_level="info"
    )
    
    logger.info("🐳 Tactile Teleop FastAPI starting in Docker container mode")
    logger.info("🌐 Behind nginx proxy - no static files served")
    logger.info("🔌 HTTP server on 0.0.0.0:8000 for container networking")
    
    # Create server in proxy mode but force HTTP mode (bypass SSL logic)
    server = FastAPIServer(cfg, behind_proxy=True, uds_path=None)
    
    # Override the server start method to force HTTP mode on port 8000
    import uvicorn
    from tactile_teleop.web_server.main import logger as web_logger
    
    # Custom HTTP-only server configuration
    uvicorn_config = uvicorn.Config(
        app=server.app,
        host=cfg.host_ip,
        port=cfg.https_port,  # Use port 8000 for HTTP
        ssl_keyfile=None,     # Force HTTP mode
        ssl_certfile=None,    # Force HTTP mode
        log_level=cfg.log_level.lower(),
        access_log=True,
        use_colors=False,
    )
    
    server.server = uvicorn.Server(uvicorn_config)
    web_logger.info(f"HTTP server starting on {cfg.host_ip}:{cfg.https_port}")
    
    # Start the server directly
    await server.server.serve()
    logger.info("✅ Web server shutdown complete.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("👋 Web server interrupted")