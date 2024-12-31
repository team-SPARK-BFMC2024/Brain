import json
from src.utils.messages.allMessages import Cars, Semaphores
from twisted.internet import protocol
from src.utils.messages.messageHandlerSender import messageHandlerSender

class udpListener(protocol.DatagramProtocol):
    """This class is used to receive the information from the servers.

    Args:
        queue (multiprocessing.queues.Queue): the queue to send the info
    """

    def __init__(self, queuesList, logger, debugging):
        self.semaphoresSender = messageHandlerSender(queuesList, Semaphores)
        self.logger = logger
        self.debugging = debugging

    def datagramReceived(self, datagram, addr):
        """Specific function for receiving the information. It will select and create different dictionary for each type of data we receive(car or semaphore)

        Args:
            datagram (dictionary): In this we store the data we get from servers.
        """
        dat = datagram.decode("utf-8")
        dat = json.loads(dat)

        if dat["device"] == "semaphore":
            tmp = {"id": dat["id"], "state": dat["state"], "x": dat["x"], "y": dat["y"]}

        elif dat["device"] == "car":
            tmp = {"id": dat["id"], "x": dat["x"], "y": dat["y"]}
        if self.debugging:
            self.logger.info(tmp)
        self.semaphoresSender.send(tmp)

    def stopListening(self):
        super().stopListening()
