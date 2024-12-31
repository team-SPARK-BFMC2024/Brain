from src.templates.threadwithstop import ThreadWithStop
from twisted.internet import reactor
from src.data.Semaphores.threads.udpListener import udpListener


class threadSemaphores(ThreadWithStop):
    """Thread which will handle processCarsAndSemaphores functionalities

    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        listenPort (int, optional): Listening port. Defaults to 5007.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logger, debugging, listenPort=5007):
        super(threadSemaphores, self).__init__()
        self.listenPort = listenPort
        self.queueList = queueList
        self.logger = logger
        self.debugging = debugging
        self.udp_factory = udpListener(self.queueList, self.logger, self.debugging)
        self.reactor = reactor
        self.reactor.listenUDP(self.listenPort, self.udp_factory)

    # ======================================= RUN ==========================================
    def run(self):
        """
        Run the thread.
        """
        self.reactor.run(installSignalHandlers=False)

    # ====================================== STOP ==========================================
    def stop(self):
        """
        Stop the thread.
        """
        self.reactor.stop()
        super(threadSemaphores, self).stop()
