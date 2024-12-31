from twisted.internet import task


class periodicTask(task.LoopingCall):
    def __init__(self, interval, shrd_mem, tcp_factory):
        super().__init__(self.periodicCheck)
        self.interval = interval
        self.shrd_mem = shrd_mem
        self.tcp_factory = tcp_factory

    def start(self):
        """
        Start the periodic task with the specified interval.
        """
        super().start(self.interval)

    def periodicCheck(self):
        """
        Perform the periodic check and send data to the server.
        """
        tosend = self.shrd_mem.get()
        for mem in tosend:
            self.tcp_factory.send_data_to_server(mem)
