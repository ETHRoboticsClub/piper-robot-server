"""CLI entry point for the `telegrip` console script.

This thin wrapper delegates to the dedicated robot- and web-server CLI helpers.
Keeping it minimal avoids duplicating argument parsing logic scattered across
those components.
"""

from telegrip.robot_server.main import main_cli as _robot_server_main_cli
from telegrip.web_server.main import main_cli as _web_server_main_cli


def main_cli() -> None:
    """Launch robot-server first, then the web-server."""
    _robot_server_main_cli()
    _web_server_main_cli()


if __name__ == "__main__":
    main_cli()
