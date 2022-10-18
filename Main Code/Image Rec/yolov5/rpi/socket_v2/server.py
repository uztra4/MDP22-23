# to be run on pc

#import datetime
#import struct
import socket
import sys
import threading
import time
import requests
import os
import subprocess


# --- constants ---
WIFI_PORT = 5001
# HOST = '192.168.45.208'   # IP address of server (PC)
HOST= '192.168.45.68'
#HOST = '172.0.0.1'

PORT = WIFI_PORT  # Local port


# --- functions ---
def exit_prog(inp):
    if inp == "exit":
        sys.exit(0)


def get_detection(url, img_path):
    r = requests.get(url + '/img_rec', files = {
        'image': open(img_path, 'rb')
    })
    
    return r.json()

results = ""
def handle_client(conn, addr):
    try:
        
        while True:
            # conn.send(results.encode())

            data = conn.recv(4096)

            # if str(data) == "b'img'":
            print("Receiving image file..")

            f = open(f'stitched.jpg', 'wb') # tbc

            while data != bytes(''.encode()):
                f.write(data)
                data = conn.recv(4096)

            print("Image received")

            # #time.sleep(2)
            # print("Carrying out Image Recognition...")
            # #res = get_detection("http://24ba-34-82-180-8.ngrok.io", 'C:/Users/chloe/Documents/Github/MDP/RPI/image.jpg')
            # results = os.popen("docker run --rm -it \
            #     -v C:/Users/chloe/Documents/MDP_AI_NEW/:/images:rshared\
            #     -v C:/Users/chloe/Documents/GitHub/MDP/RPI/:/input:rshared\
            #         fb8433fada44 python /images/img_rec.py /input/{}"\
            #             .format("image.jpg")).read()
            
            # print(type(results))
            # results = str(results).replace('Premature', '')\
            #     .replace('end', '')\
            #     .replace('of', '')\
            #     .replace('file', '')\
            #     .replace('JPEG', '')\
            #     .strip() 
            # with open('results.json', 'w') as f:
            #     f.write(results)
            
            # subprocess.run(["scp", "-i", "~/Documents/pi.pem",
            # "C:\\Users\\chloe\\Documents\\GitHub\\MDP\\RPI\\results.json",
            # "pi@192.168.29.29:/home/pi/codes/RPI/results.json"])

            # print(results)
            

            #results = res[0]['pred_classes']  

            # data = str(datetime.datetime.now()).encode()
            # conn.sendall((res[0]['pred_classes']).encode())
            # print('send: ', data)
            # print('---sleep---')
            #time.sleep(3)
            conn.close()
            

    except BrokenPipeError:
        print('[DEBUG] addr:', addr, 'Connection closed by client?')
    except Exception as ex:
        print('[DEBUG] addr:', addr, 'Exception:', ex, )
    except KeyboardInterrupt as ex:
        conn.close()
        print('END')
    finally:
        conn.close()

# --- main ---


try:
    # --- create socket ---
    print("Starting server")
    print('[DEBUG] create socket')
    results = ""

    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s = socket.socket()  # default value is (socket.AF_INET, socket.SOCK_STREAM) so you don't have to use it

    # solution for "[Error 89] Address already in use". Use before bind()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # --- assign socket to local IP (local NIC) ---

    print('[DEBUG] bind:', (HOST, PORT))

    s.bind((HOST, PORT))  # one tuple (HOST, PORT), not two arguments

    # --- set size of queue ---

    print('[DEBUG] listen')

    s.listen(1)  # number of clients waiting in queue for "accept".
                 # If queue is full then client can't connect.

    while True:
        # --- accept client ---

        # accept client and create new socket `conn` (with different port) for this client only
        # and server will can use `s` to accept other clients (if you will use threading)
        print('[DEBUG] Accept ... waiting')

        conn, addr = s.accept()  # socket, address

        print('[DEBUG] Connected by:', addr)

        t = threading.Thread(target=handle_client, args=(conn, addr))
        t.start()


except Exception as ex:
    print(ex)
except KeyboardInterrupt as ex:
    print(ex)
except:
    print(sys.exc_info())
finally:
    # --- close socket ---
    print('[DEBUG] close socket')
    s.close()