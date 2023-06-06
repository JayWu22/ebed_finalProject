from time import sleep
import erpc
from finalbbcar import *
import sys
import numpy as np
import struct

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Usage: python led_test_client.py <serial port to use>")
        exit()

    # Initialize all erpc infrastructure
    xport = erpc.transport.SerialTransport(sys.argv[1], 9600)
    client_mgr = erpc.client.ClientManager(xport, erpc.basic_codec.BasicCodec)
    client = client.bbcarServiceClient(client_mgr)

    while True:
        command = input()
        if command == "d":
            temp = client.bbcar_distance()
            print(temp)
        elif command == "s":
            temp = client.bbcar_speed()
            print(temp)
        sleep(1)
        