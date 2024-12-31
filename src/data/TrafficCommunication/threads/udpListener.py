from twisted.internet import protocol
import src.data.TrafficCommunication.useful.keyDealer as keyDealer

class udpListener(protocol.DatagramProtocol):
    """This class will handle the connection.

    Args:
        decrypt_key (String): A path to the decripting key.
        serverfound (function): This function will be called if the server will be found.
    """

    def __init__(self, decrypt_key, serverfound):
        decrypt_key = decrypt_key
        self.pub_key = keyDealer.load_public_key(decrypt_key)
        self.serverfoundCllback = serverfound

    def startProtocol(self):
        print("Looking for Traffic Communicaiton Server")

    def datagramReceived(self, datagram, address):
        """In this function we split the receive data and we call the callbackfunction"""
        
        try:
            dat = datagram.split(b"(-.-)")
            if len(dat) != 2:
                raise Exception("Plaintext or signature not present")
            a = keyDealer.verify_data(self.pub_key, dat[1], dat[0])
            if not a:
                raise Exception("signature not valid")
            msg = dat[1].decode().split(":")
            port = int(msg[1])
            self.serverfoundCllback(address[0], port)
        except Exception as e:
            print("TrafficCommunication -> udpListener -> datagramReceived:")
            print(e)

    def stopListening(self):
        self.transport.stopListening()
