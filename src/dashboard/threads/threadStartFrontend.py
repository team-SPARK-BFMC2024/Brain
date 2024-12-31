if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import subprocess
import os
from src.templates.threadwithstop import ThreadWithStop

class ThreadStartFrontend(ThreadWithStop):

    # ================================ INIT ===============================================

    def __init__(self, logger, project_path=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend")):
        """Thread for managing an Angular development server.
        
        Args:
            project_path (str): The file path to the Angular project directory.
        """
        
        self.project_path = project_path
        self.logger= logger
        super().__init__()
    
    # ================================= RUN ===============================================

    def run(self):
        """Overrides the Thread.run. Starts the Angular server and monitors the _running flag."""

        try:
            subprocess.run(f"cd {self.project_path} && npm start", shell=True, check=True)
            self.logger.info("Angular server started successfully.")
        except subprocess.CalledProcessError as e:
            self.logger.warning(f"Failed to start the Angular development server: {e}")

    # ================================ STOP ===============================================

    def stop(self):
        """Stops the Angular development server if running."""

        self.logger.warning("Angular server stopped.")

# ================================= EXAMPLE ===============================================

if __name__ == "__main__":
    # Replace '../frontend/' with the actual path to your Angular project
    angular_thread = ThreadStartFrontend("../frontend/")
    angular_thread.start()
    input("Press Enter to stop the server...")
    angular_thread.stop()
    angular_thread.join()
