import socket, sys, struct

try:
    addr = sys.argv[1]
    rate = int(sys.argv[2])
except (ValueError, IndexError):
    print 'usage: setparams.py ipaddr rate'
else:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ndata = 2  # number of words
    pkt = struct.pack('<HHHHHBBHHHH' + 'I',
        10 + ndata,
        0x8000,
        10,
        0,
        0xF1F0,
        0,
        0,
        0,
        0,
        0,
        0,
        rate,
    )
    s.sendto(pkt, (addr, 54321))
    s.recvfrom(1024)
    print 'ok'
