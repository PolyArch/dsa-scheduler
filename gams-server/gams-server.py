

import socket
import os

HOST = ''
PORT = 20202

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)


while 1:
    conn, addr = s.accept()
    print 'Connected by', addr

    data = conn.recv(1024)
    if not data:
        break
    if data == "__END__":
        break
    print data
    if data == "run-gams":
        data = conn.recv(1024)

        if os.path.exists(data):
            dname = os.path.dirname(data)
            fname = os.path.basename(data)
            os.system("cd %s && gams %s"%(dname, fname))
            conn.send("__DONE__")
        else:
            conn.send("__ERROR__")
    else:
        print "Wrong command"
    conn.close()

s.close()
