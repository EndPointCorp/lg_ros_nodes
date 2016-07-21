from appctl_support import ProcController


class TCPRelay(ProcController):
    """Relays messages from one socket to another."""
    def __init__(self, local_port, remote_port):
        cmd = ['/usr/bin/socat']
        cmd.append(
            'TCP4-LISTEN:{},fork,reuseaddr,bind=0.0.0.0'.format(remote_port)
        )
        cmd.append(
            'TCP4:127.0.0.1:{}'.format(local_port)
        )

        super(TCPRelay, self).__init__(cmd=cmd)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
