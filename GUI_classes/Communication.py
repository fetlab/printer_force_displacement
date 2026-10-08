import multiprocessing
import queue
from pathlib import Path

class Communication:
    def __init__(self, path_to_TestResults: Path):
        self.msg_queue = multiprocessing.Queue()
        self.cmd_queue = multiprocessing.Queue()
        # Stores test data for plotting of all tests run in the current session
        self.tests = {}
        # Counter for number of tests run in the current session
        self.number_of_tests = 0
        self.path_to_TestResults = path_to_TestResults
    
    def check_messages(self):
        """reads messages from the testing process(msg_queue) and updates state accordingly"""
        try:
            while True:
                msg = self.msg_queue.get_nowait()
                
                if isinstance(msg, str) and msg == "done":
                    # self.stop_event.set()
                    print("Process reported done!")

                if isinstance(msg, str) and msg == "connected":
                    self.number_of_tests += 1
                
                # Handle structured test result messages without importing at module import time
                if hasattr(msg, "displacement") and hasattr(msg, "force"):
                    testno = self.number_of_tests

                    if testno not in self.tests:
                        active = getattr(self, "active_config", None)
                        self.tests[testno] = {"x": [], "y": [], "config_name": active.stem if active else "unknown"}

                    # Only plot valid data
                    if msg.displacement != float('inf') and msg.force != float('inf'):
                        self.tests[testno]["x"].append(abs(msg.displacement))
                        self.tests[testno]["y"].append(abs(msg.force))
                        
        except queue.Empty:
            pass
        
    def send_cmd_msg(self, msg):
        """sends command messages to the testing process(cmd_queue)"""
        self.cmd_queue.put(msg)

    def send_msg(self, msg):
        """sends messages to the testing process(msg_queue)"""
        self.msg_queue.put(msg)
    
   

