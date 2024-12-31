# Import necessary modules

from twisted.internet import reactor
from src.templates.threadwithstop import ThreadWithStop
from src.data.TrafficCommunication.threads.udpListener import udpListener
from src.data.TrafficCommunication.threads.tcpClient import tcpClient
from src.data.TrafficCommunication.useful.periodicTask import periodicTask


class threadTrafficCommunication(ThreadWithStop):
    """Thread which will handle processTrafficCommunication functionalities

    Args:
        shrd_mem (sharedMem): A space in memory for mwhere we will get and update data.
        queuesList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        deviceID (int): The id of the device.
        decrypt_key (String): A path to the decription key.
    """

    # ====================================== INIT ==========================================
    def __init__(self, shrd_mem, queueslist, deviceID, frequency, decrypt_key):
        super(threadTrafficCommunication, self).__init__()
        self.listenPort = 9000
        self.queue = queueslist["General"]

        self.tcp_factory = tcpClient(self.serverLost, deviceID, frequency, self.queue) # Handles the connection with the server

        self.udp_factory = udpListener(decrypt_key, self.serverFound) #Listens for server broadcast and validates it

        self.period_task = periodicTask(1, shrd_mem, self.tcp_factory) # Handles the Queue of errors accumulated so far.

        self.reactor = reactor
        self.reactor.listenUDP(self.listenPort, self.udp_factory)

    # =================================== CONNECTION =======================================
    def serverLost(self):
        """If the server disconnects, we stop the factory listening and start the reactor listening"""

        self.reactor.listenUDP(self.listenPort, self.udp_factory)
        self.tcp_factory.stopListening()
        self.period_task.stop()

    def serverFound(self, address, port):
        """If the server was found, we stop the factory listening, connect the reactor, and start the periodic task"""
        
        self.reactor.connectTCP(address, port, self.tcp_factory)
        self.udp_factory.stopListening()
        self.period_task.start()



    # ======================================= RUN ==========================================
    def run(self):
        self.reactor.run(installSignalHandlers=False)

    # ====================================== STOP ==========================================
    def stop(self):
        self.reactor.stop()
        super(threadTrafficCommunication, self).stop()
