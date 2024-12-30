if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.hardware.Autonomous.threads.threadAutonomous import threadAutonomous

class ProcessAutonomous(WorkerProcess):
    """This process handles Autonomous.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(ProcessAutonomous, self).__init__(self.queuesList)

    def run(self):
        """Apply the initializing methods and start the threads."""
        super(ProcessAutonomous, self).run()

    def _init_threads(self):
        """Create the Autonomous Publisher thread and add to the list of threads."""
        AutonomousTh = threadAutonomous(
            self.queuesList, self.logging, self.debugging
        )
        self.threads.append(AutonomousTh)
