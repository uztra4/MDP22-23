import sys

from rpi_client import RPiClient
from rpi_server import RPiServer


def main():
    # Create a server for the PC to connect to
    server = RPiServer("192.168.8.8", 4160)
    # Wait for the PC to connect to the RPi.
    print("Waiting for connection from PC...")
    try:
        server.start()
    except Exception as e:
        print(e)
        server.close()
        sys.exit(1)
    print("Connection from PC established!\n")

    # Then, we use this to connect to the PC's server.
    host = server.address[0]
    # Create a client to connect to the PC's server.
    try:
        client = RPiClient(host, 4161)
    except OSError as e:
        print(e)
        server.close()
        sys.exit(1)

    # Wait to connect to RPi.
    print(f"Attempting connection to PC at {host}:{4161}")
    while True:
        try:
            client.connect()
            break
        except OSError:
            pass
        except Exception as e:
            print(e)
            server.close()
            client.close()
            sys.exit(1)
    print("Connected to PC!\n")

    # Send over the obstacle data to the PC.
    print("Sending obstacle data to PC...")
    # TODO: Send actual obstacle data to the PC.
    obstacle_data = [[23, 23, 90], [45, 45, 78]]
    client.send_message(obstacle_data)
    client.close()
    print("Done!\n")

    # Receive commands from the PC.
    print("Receiving robot commands from PC...")
    try:
        commands = server.receive_data()
        print("Commands received!\n")
        print(commands)
    except Exception as e:
        print(e)
    finally:
        server.close()


if __name__ == '__main__':
    main()
