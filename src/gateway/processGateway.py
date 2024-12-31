if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../..")

from src.templates.workerprocess import WorkerProcess
from src.gateway.threads.threadGateway import threadGateway


class processGateway(WorkerProcess):
    """This process handle all the data distribution\n
    Args:
        queueList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logger, debugging=False):
        self.logger = logger
        self.debugging = debugging
        super(processGateway, self).__init__(queueList)

    # ===================================== RUN ===========================================
    def run(self):
        """Apply the initializing methods and start the threads."""

        super(processGateway, self).run()

    # ===================================== INIT TH ==========================================
    def _init_threads(self):
        """Initializes the gateway thread."""
        
        gatewayThread = threadGateway(self.queuesList, self.logger, self.debugging)
        self.threads.append(gatewayThread)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE Owner HERE  ++
#                  in terminal:    python3 processGateway.py

if __name__ == "__main__":
    from multiprocessing import Pipe, Queue, Event
    import time
    import logging

    allProcesses = list()
    # We have a list of multiprocessing.Queue() which individualy represent a priority for processes.
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    logging = logging.getLogger()
    process = processGateway(queueList, logging, debugging=True)
    process.daemon = True
    process.start()

    pipeReceive1, pipeSend1 = Pipe()
    queueList["Config"].put(
        {
            "Subscribe/Unsubscribe": "suBsCribe",
            "Owner": "Camera",
            "msgID": 1,
            "To": {"receiver": 1, "pipe": pipeSend1},
        }
    )
    time.sleep(1)

    pipeReceive2, pipeSend2 = Pipe()
    queueList["Config"].put(
        {
            "Subscribe/Unsubscribe": "Subscribe",
            "Owner": "Camera",
            "msgID": 2,
            "To": {"receiver": 2, "pipe": pipeSend2},
        }
    )
    time.sleep(1)

    pipeReceive3, pipeSend3 = Pipe()
    queueList["Config"].put(
        {
            "Subscribe/Unsubscribe": "subscribe",
            "Owner": "Camera",
            "msgID": 3,
            "To": {"receiver": 3, "pipe": pipeSend3},
        }
    )
    time.sleep(1)

    queueList["Critical"].put(
        {
            "Owner": "Camera",
            "msgID": 1,
            "msgType": "1111",
            "msgValue": "This is the text1",
        }
    )

    queueList["Warning"].put(
        {
            "Owner": "Camera",
            "msgID": 3,
            "msgType": "1111",
            "msgValue": "This is the text3",
        }
    )

    queueList["General"].put(
        {
            "Owner": "Camera",
            "msgID": 2,
            "msgType": "1111",
            "msgValue": "This is the text2",
        }
    )
    time.sleep(2)

    # Code to verify that the function send Owner threadGateway.py is working properly.

    print(pipeReceive3.recv())
    print(pipeReceive1.recv())
    print(pipeReceive2.recv())

    # ===================================== STAYING ALIVE ====================================

    process.stop()
