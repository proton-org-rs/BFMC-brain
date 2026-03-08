"""
Decision Making Process

This process encapsulates the Decision Making thread which is responsible for:
- Subscribing to detection events (traffic signs, objects, traffic lights)
- Using Path Planning to analyze upcoming road features
- Determining the appropriate driving mode (ROUNDABOUT, CROSSING, etc.)
- Publishing driving mode changes for the AutonomousDriving component

The process follows the standard BFMC architecture:
    processDecisionMaking (WorkerProcess)
        └── threadDecisionMaking (ThreadWithStop)
                ├── PathPlanning
                │       └── GraphMap (Competition_track_graph.graphml)
                └── DrivingModeContext

Args:
    queueList: Dictionary of multiprocessing.queues.Queue for messaging.
    logging: Logger object for debugging.
    graphml_path: Path to the competition track GraphML file.
    ready_event: Event to signal when the process is ready.
    debugging: Enable verbose debug logging.
"""

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import os
from src.templates.workerprocess import WorkerProcess
from src.DecisionMaking.threads.threadDecisionMaking import threadDecisionMaking


class processDecisionMaking(WorkerProcess):
    """
    Process wrapper for the Decision Making component.
    
    Manages the threadDecisionMaking lifecycle within the BFMC
    multiprocessing architecture. The process is started/stopped
    based on system mode transitions in main.py.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, graphml_path, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.graphml_path = graphml_path
        super(processDecisionMaking, self).__init__(self.queuesList, ready_event)

    # ===================================== INIT THREADS ===================================
    def _init_threads(self):
        """Create the Decision Making thread and add to the thread list."""
        decision_thread = threadDecisionMaking(
            self.queuesList,
            self.logging,
            self.graphml_path,
            self.debugging,
        )
        self.threads.append(decision_thread)


# =================================== STANDALONE TEST ======================================

if __name__ == "__main__":
    from multiprocessing import Queue, Event
    import logging
    import time

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Log": Queue(),
    }

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()

    graphml_path = os.path.join(
        os.path.dirname(__file__), "..", "..", "Competition_track_graph.graphml"
    )

    ready = Event()
    process = processDecisionMaking(queueList, logger, graphml_path, ready, debugging=True)
    process.start()

    print("Decision Making process started. Waiting for ready...")
    ready.wait(timeout=5)
    print("Ready! Press Ctrl+C to stop.")

    try:
        process.join()
    except KeyboardInterrupt:
        print("\nStopping Decision Making...")
        process.stop()
