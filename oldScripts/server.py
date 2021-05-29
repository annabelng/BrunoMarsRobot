import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), 1234))
s.listen(5)

clientsocket, address = s.accept()
print(f"Connection from {address} has been established.")

run = True
while run:
    msg = clientsocket.recv(1024)
    command = msg.decode("utf-8")
    print("command = "+command+str(len(command)))
    if len(command) == 0:
        run = False

clientsocket.close()