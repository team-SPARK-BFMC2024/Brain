from src.templates.threadwithstop import ThreadWithStop

class threadGateway(ThreadWithStop):
    """Thread which will handle processGateway functionalities.\n
    Args:
        queuesList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ===================================== INIT =========================================

    def __init__(self, queueList, logger, debugging):
        super(threadGateway, self).__init__()
        self.logger = logger
        self.debugging = debugging
        self.sendingList = {}
        self.queuesList = queueList
        self.messageApproved = []

    # =================================== SUBSCRIBE ======================================

    def subscribe(self, message):
        """This functin will add the pipe into the approved messages list and it will be added into the dictionary of sending
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """
        
        # Declaration of variables:
        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]
        Pipe = message["To"]["pipe"]
        print(Owner, Id, To, Pipe)
        if not Owner in self.sendingList.keys():
            self.sendingList[Owner] = {}
        if not Id in self.sendingList[Owner].keys():
            self.sendingList[Owner][Id] = {}
        if not To in self.sendingList[Owner][Id].keys():
            self.sendingList[Owner][Id][To] = Pipe
        self.messageApproved.append((Owner, Id))
        # Debugging( you can comment this):
        if self.debugging:
            self.printList()

    # ================================== UNSUBSCRIBE =====================================

    def unsubscribe(self, message):
        """This functin will remove the pipe into the approved messages list and it will be added into the dictionary of sending
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        Owner = message["Owner"]
        Id = message["msgID"]
        To = message["To"]["receiver"]

        # We delete the value from Dictionary
        del self.sendingList[Owner][Id][To]
        self.messageApproved.remove((Owner, Id))
        if self.debugging:
            self.printList()

    # =================================== SENDING ========================================

    def send(self, message):
        """This functin will send the message on all the pipes that are in the sending list of the message ID.
        Args:
            message(dictionary): Dictionary received from the multiprocessing queues ( the config one).
        """

        Owner = message["Owner"]
        Id = message["msgID"]
        Type = message["msgType"]
        Value = message["msgValue"]
        if (Owner, Id) in self.messageApproved:
            for element in self.sendingList[Owner][Id]:
                # We send a dictionary that contain the type of the message and message
                self.sendingList[Owner][Id][element].send(
                    {"Type": Type, "value": Value, "id": Id, "Owner": Owner}
                )
                if self.debugging:
                    self.logger.warning(message)

    # ====================================================================================

    # Function for debugging:
    def printList(self):
        """Made for debugging"""

        self.logger.warning(self.sendingList)

    # ==================================== RUN ===========================================

    def run(self):
        """This function will take the messages in priority order form the queues.\n
        the prioirty is: Critical > Warning > General
        """
        
        while self._running:
            message = None
            # We are using "elif" because we are processing one message at a time.
            # We work with the queues in the priority order( We start from the high priority to low priority)
            if not self.queuesList["Critical"].empty():
                message = self.queuesList["Critical"].get()
            elif not self.queuesList["Warning"].empty():
                message = self.queuesList["Warning"].get()
            elif not self.queuesList["General"].empty():
                message = self.queuesList["General"].get()
            if message is not None:
                self.send(message)
            if not self.queuesList["Config"].empty():
                message2 = self.queuesList["Config"].get()
                if str.lower(message2["Subscribe/Unsubscribe"]) == "subscribe":
                    self.subscribe(message2)
                else:
                    self.unsubscribe(message2)


# =====================================================================================
