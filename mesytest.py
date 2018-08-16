import os
import sys
import socket
import time
import threading

try:
    addr = sys.argv[1]
    rate = int(sys.argv[2])
    pktsize = int(sys.argv[3])
    dt = float(sys.argv[4])
except:
    print 'usage: mesytest.py ipaddr rate minperpacket meastime'
    sys.exit(1)

os.system('python mesyparams.py %s %s %s' % (addr, rate, pktsize))

n = [0, 0]

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 54321))
s.sendto('\x00\x00\x00\x01\x00\x00\x00\x00' + '\x01\x00' + '\x00' * 10, (addr, 54321))
s.recvfrom(2000)

def get():
    while True:
        d, _ = s.recvfrom(2000)
        if d[3] == '\x01':
            continue
        n[0] += 1
        blen = ord(d[0]) | (ord(d[1]) << 8)
        n[1] += (blen - 21) / 3

t = threading.Thread(target=get)
t.setDaemon(True)
t.start()
time.sleep(dt)
s.sendto('\x00\x00\x00\x01' + '\x00' * 16, (addr, 54321))
time.sleep(0.1)
print 'got:', n[0], 'packets with', n[1], 'events =>', n[1]/dt, 'ev/s'
