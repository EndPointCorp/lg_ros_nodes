class QueryWriter:
    def __init__(self, filename):
        self.filename = filename

    def handle_flyto_kml(self, msg):
        kml = msg.data
        query = 'flytoview={}'.format(kml)
        with open(self.filename, 'w') as f:
            f.write(query)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
