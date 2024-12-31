if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.data.Semaphores.threads.threadSemaphores import (
    threadSemaphores,
)

class processSemaphores(WorkerProcess):
    """This process will receive the location of the other cars and the location and the state of the semaphores.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging = False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processSemaphores, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""

        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processSemaphores, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""

        super(processSemaphores, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the thread and add to the list of threads."""

        CarsSemTh = threadSemaphores(self.queuesList, self.logging, self.debugging)
        self.threads.append(CarsSemTh)


# =================================== EXAMPLE =========================================

if __name__ == "__main__":
    from multiprocessing import Queue

    queueList = {
        "Critical": Queue(),  # Queue for critical messages
        "Warning": Queue(),  # Queue for warning messages
        "General": Queue(),  # Queue for general messages
        "Config": Queue(),  # Queue for configuration messages
    }

    allProcesses = list()
    process = processSemaphores(queueList)
    process.start()

    x = range(6)
    for n in x:
        print(queueList["General"].get())  # Print general messages

    process.stop()
