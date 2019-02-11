import  socket

class Multicast():
    def __init__(self, group_addr, port):
        bind_addr = '0.0.0.0'

        # Create a IPv4/UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Avoid error 'Address already in use'.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Construct a membership_request
        membership_request = socket.inet_aton(group_addr) + socket.inet_aton(bind_addr)

        # Send add membership request to socket
        self.sock.setsockopt(socket.IPPROTO_IP, 
                socket.IP_ADD_MEMBERSHIP, membership_request)

        # Bind the socket to an interfaces
        self.sock.bind((bind_addr, port))

        # Set non-blocking receiving mode
        self.sock.setblocking(False)


    def recv(self, buf_length):
        try:
            buf = self.sock.recv(buf_length)

        except:
            return  False

        return  buf


    def close(self):
        self.sock.close()
