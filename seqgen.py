import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', 12345))
s.sendto('start', ('192.168.1.191', 50000))
t0 = None
ll = []
pktl = 0

for i in range(10*1024):
    b, _ = s.recvfrom(2000)
    if t0 is None:
        print 'start'
        pktl = len(b)
        t0 = time.time()
    n = ord(b[0])<<24 | ord(b[1])<<16 | ord(b[2])<<8 | ord(b[3])
    ll.append(n)
s.sendto('stop', ('192.168.1.191', 50000))
t1 = time.time()
print 'Took %.3f s (%.1f MB/s)' % (t1-t0, 10.*(pktl/1024.)/(t1-t0))
assert ll == range(10*1024)
