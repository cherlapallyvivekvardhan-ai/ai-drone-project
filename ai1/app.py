"""Launcher: python app.py  ->  starts the server and opens the dashboard."""
import argparse
import socket
import sys
import threading
import webbrowser
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from backend.server import app  # noqa: E402


def free_port(preferred, host="127.0.0.1"):
    for port in range(preferred, preferred + 20):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex((host, port)) != 0:
                return port
    raise RuntimeError("No free port found near %d." % preferred)


def main():
    p = argparse.ArgumentParser(description="AI drone path planning dashboard")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--no-browser", action="store_true")
    p.add_argument("--debug", action="store_true")
    a = p.parse_args()

    port = free_port(a.port, a.host)
    url = f"http://{a.host}:{port}"
    print(f"\n  Drone mission platform running at {url}\n  Press Ctrl+C to stop.\n")
    if not a.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    app.run(host=a.host, port=port, debug=a.debug, use_reloader=False)


if __name__ == "__main__":
    main()
