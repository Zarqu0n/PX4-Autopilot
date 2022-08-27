"""
Zarquon
"""

import socket, cv2, pickle, struct, imutils


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = '192.168.190.129'#server ip
port = 5050
print('Server {0}:{1} adresinde kuruldu'.format(host_ip,port))
socket_address = (host_ip, port)

server_socket.bind(socket_address)

server_socket.listen(5)

print(socket_address,' adresinde dinleniyor.')

# Socket Accept
while True:
    client_socket, addr = server_socket.accept()
    print("----->Bağlant, saglandi:",addr)

    if client_socket:
        vid = cv2.VideoCapture(0)
        data = b""
        payload_size = struct.calcsize("Q")
        while (vid.isOpened()):

            #Client'ten alınan verinin okunması

            """  while len(data) < payload_size:
                packet = client_socket.recv(4*1024) # 4K
                if not packet: break
                data+=packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q",packed_msg_size)[0]
            
            while len(data) < msg_size:
                data += client_socket.recv(4*1024)
            frame_data = data[:msg_size]
            data  = data[msg_size:]
            frame = pickle.loads(frame_data)

            dataFromServer = client_socket.recv(1024).decode()
            print(dataFromServer)
"""
            img, frame = vid.read()
            frame = imutils.resize(frame, width=320)
            a = pickle.dumps(frame)
            message = struct.pack("Q", len(a)) + a
            client_socket.sendall(message)
            cv2.imshow('Server frame', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                client_socket.close()