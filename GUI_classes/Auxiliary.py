from pathlib import Path
import json
import sys
#sys.path.insert(0, str(Path(__file__).parent.parent))
from force_test import main, ArgumentError
from .DataJson import TestConfig

class Auxiliary:
    def json_encode(obj):
        if hasattr(obj, "to_json_encodable"):
            return obj.to_json_encodable()
        raise TypeError

    def load_config(path: Path) -> TestConfig:
        with open(path) as f:
            data = json.load(f)
        return TestConfig(**data)
    
    def safe_call(stop_event, queue, cmd_queue, config):
        """Calls main() in a safe way, catching exceptions"""
        if stop_event == True:
            print("thread stopped")
            return
        
        old_exit = sys.exit
        try:
            main(queue,cmd_queue,config)
        
        except ArgumentError as e:
            print("ArgumentError in main():", e)
        
        except SystemExit as e:
            print("SystemExit in main():", e)
        
        except Exception as e:
            print("Exception in main():", e)
        
        finally:
            sys.exit = old_exit