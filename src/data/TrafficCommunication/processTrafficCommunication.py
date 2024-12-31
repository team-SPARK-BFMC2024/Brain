if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

# Import necessary modules
from multiprocessing import Pipe
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.templates.workerprocess import WorkerProcess
from src.data.TrafficCommunication.threads.threadTrafficCommunicaiton import (
    threadTrafficCommunication,
)
class processTrafficCommunication(WorkerProcess):
    """This process receives the location of the car and sends it to the processGateway.
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Used for debugging.
        deviceID (int): The ID of the device.
        frequency (float): The frequency of communication.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, deviceID, debugging, frequency=1):
        self.queuesList = queueList
        self.logging = logging
        self.shared_memory = sharedMem()
        self.filename = "src/data/TrafficCommunication/useful/publickey_server_test.pem"
        self.deviceID = deviceID
        self.frequency = frequency
        self.debugging = debugging
        super(processTrafficCommunication, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        
        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processTrafficCommunication, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""

        super(processTrafficCommunication, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Traffic Communication thread and add it to the list of threads."""

        TrafficComTh = threadTrafficCommunication(
            self.shared_memory, self.queuesList, self.deviceID, self.frequency, self.filename
        )
        self.threads.append(TrafficComTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processTrafficCommunication.py

if __name__ == "__main__":
    from multiprocessing import Queue
    import time

    shared_memory = sharedMem()
    locsysReceivePipe, locsysSendPipe = Pipe(duplex=False)
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    # filename = "useful/publickey_server.pem"
    filename = "useful/publickey_server_test.pem"
    deviceID = 3
    frequency = 0.4
    traffic_communication = threadTrafficCommunication(
        shared_memory, queueList, deviceID, frequency, filename
    )
    traffic_communication.start()    

    start_time = time.time()
    duration = 10  # specify the duration in seconds
    
    shared_memory.insert("devicePos", [1.2, 2.3]) # send a position to the server
    shared_memory.insert("deviceRot", [3.4]) # send a rotation to the server
    shared_memory.insert("deviceSpeed", [4.5]) # send a speed to the server
    shared_memory.insert("historyData", [5.6, 6.7, 8]) # send a history data point to the server

    while time.time() - start_time < duration:
        try:
            print(queueList["General"].get(timeout=1))
        except:pass
    traffic_communication.stop()
