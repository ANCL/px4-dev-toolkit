"""Relay one remote MAVLink UDP connection to the toolkit's local clients.

Accept telemetry only from the configured remote IP, excluding packets from
other vehicles on the same network. Forward telemetry to the local shell
and transfer tools; return their MAVLink commands to the remote UDP endpoint.
"""

import ipaddress
import socket
import sys

SHELL_PORT = 14661
TRANSFER_PORT = 14662
LOCAL_CLIENTS = (SHELL_PORT, TRANSFER_PORT)


def main():
    if len(sys.argv) != 4:
        raise SystemExit("Usage: mavlink_udp_relay.py REMOTE_IP REMOTE_PORT LISTEN_PORT")

    radio_ip = str(ipaddress.IPv4Address(sys.argv[1]))
    radio_port = int(sys.argv[2])
    listen_port = int(sys.argv[3])

    if any(port <= 0 or port > 65535 for port in (radio_port, listen_port)):
        raise SystemExit("UDP ports must be in the range 1..65535")
    if listen_port in LOCAL_CLIENTS:
        raise SystemExit("MAVLink receive port conflicts with a local toolkit port")

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as transport:
        transport.bind(("0.0.0.0", listen_port))
        print(f"MAVLink remote: {radio_ip}:{radio_port}; listening on UDP {listen_port}", flush=True)
        print("Local shell: 127.0.0.1:14661; transfers: 127.0.0.1:14662", flush=True)

        while True:
            payload, source = transport.recvfrom(65535)

            if source[0] == radio_ip:
                # Forward only the selected remote endpoint's telemetry.
                for port in LOCAL_CLIENTS:
                    transport.sendto(payload, ("127.0.0.1", port))

            elif source[0] == "127.0.0.1" and source[1] in LOCAL_CLIENTS:
                # Return local commands to the configured remote endpoint.
                transport.sendto(payload, (radio_ip, radio_port))


if __name__ == "__main__":
    main()
