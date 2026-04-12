class Address:
    __slots__ = ("host", "port")

    def __init__(self, host, port):
        self.host = host
        self.port = port
