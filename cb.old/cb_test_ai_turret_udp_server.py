import argparse
import base64
import json
import socket
import sys
import time
from pathlib import Path
import urllib.parse  # added

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-H", "--host", default="192.168.0.21")
    ap.add_argument("-p", "--port", type=int, default=5001)
    ap.add_argument("-t", "--timeout", type=float, default=2.0)
    ap.add_argument("-c", "--count", type=int, default=1, help="number of requests (0=loop)")
    ap.add_argument("--hz", type=float, default=5.0, help="request rate if looping")
    ap.add_argument("--save", action="store_true", help="save jpg if server returns base64 image")
    args = ap.parse_args()

    addr = (args.host, args.port)
    period = 1.0 / max(1.0, args.hz)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(args.timeout)

    # URL-like request payload
    payload = urllib.parse.urlencode({"req": "mp-pose"}).encode("utf-8")

    n = 0
    try:
        while True:
            t0 = time.time()
            sock.sendto(payload, addr)
            try:
                resp, remote = sock.recvfrom(65535)
                rtt_ms = (time.time() - t0) * 1000.0
            except socket.timeout:
                print("timeout")
                if args.count and n + 1 >= args.count:
                    break
                time.sleep(period)
                continue

            text = resp.decode("utf-8", errors="replace")
            try:
                data = json.loads(text)
                print(f"{data} rtt={rtt_ms:.1f} ms")
            except Exception:
                print(f"non-JSON response from {remote}: {text[:120]}...")
                if args.count and n + 1 >= args.count:
                    break
                time.sleep(period)
                continue

            n += 1
            if args.count and n >= args.count:
                break

            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    finally:
        sock.close()

if __name__ == "__main__":
    main()