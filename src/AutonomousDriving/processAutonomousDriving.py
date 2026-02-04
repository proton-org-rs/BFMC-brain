# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.data.AutonomousDriving.threads.threadAutonomousDriving import threadAutonomousDriving


class processAutonomousDriving(WorkerProcess):
    """This process handles autonomous driving in LEGACY mode.
    
    The autonomous driving logic:
    - Drives forward at constant speed
    - Monitors IR sensor for stop line detection
    - When stop line detected: stops for 3 seconds
    - Resumes driving and ignores stop lines for 5 seconds (to cross all lines)
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        ready_event (multiprocessing.Event, optional): Event to signal when process is ready.
        debugging (bool): Enable debug logging.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processAutonomousDriving, self).__init__(self.queuesList, ready_event)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the thread and add to the list of threads."""
        autonomous_thread = threadAutonomousDriving(
            self.queuesList, 
            self.logging, 
            self.debugging
        )
        self.threads.append(autonomous_thread)


# =================================== EXAMPLE =========================================

if __name__ == "__main__":
    from multiprocessing import Queue
    import logging

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Log": Queue(),
    }

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()

    process = processAutonomousDriving(queueList, logger, debugging=True)
    process.start()

    print("Autonomous driving process started. Press Ctrl+C to stop.")
    
    try:
        process.join()
    except KeyboardInterrupt:
        print("\nStopping process...")
        process.stop()
