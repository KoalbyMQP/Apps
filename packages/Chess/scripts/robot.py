## Future implementation needed. This will be where the target / user code is selected, and subsequently run.

## Current target: build.sh

from typing import Callable
import threading
from multiprocessing import Process
from cyberonics_py import Robot, Device, Target
from cyberonics_py.graphics import Button, GraphicCell
import subprocess
import os
import signal


class Finley(Robot):
    def __init__(self):
        self.control_cell = ControlCell()
        super().__init__([self.control_cell], [Chess(self)])


class ControlCell(Device):
    def __init__(self):

        self.listeners: [Callable] = []

        def button_pressed():
            print("Button pressed!")
            for listener in self.listeners:
                listener()

        button = Button(text="Press me", onclick=button_pressed)
        super().__init__(properties=[], graphic_cell=GraphicCell([button]))

    def listen(self, listener: Callable):
        self.listeners.append(listener)


class Chess(Target):
    def __init__(self, robot: Robot):
        super().__init__("Chess", robot)
        self.process = None

    def _run(self):
        # Run script
        print("Running shell script with Popen")

        script_path = os.path.join(os.path.dirname(__file__), "build.sh")

        ## Since sh is not an executable by default, must run it through bash
        self.process = subprocess.Popen(
            ["bash", script_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid
        )

        threading.Thread(target=self._read_output, args=(self.process.stdout, self.stdout_callback), daemon=True).start()
        threading.Thread(target=self._read_output, args=(self.process.stderr, self.stderr_callback), daemon=True).start()

        return threading.current_thread()

    async def _shutdown(self, beat):
        if self.process:
            ## Terminate the process group
            pgid = os.getpgid(self.process.pid)
            os.killpg(pgid, signal.SIGTERM)
            ## If process ignores sigterm, force the shutdown by killing the process
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
