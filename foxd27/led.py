class LED:
    def __init__(self, name, initial_state=0):
        self._name = name
        self._state = initial_state
        self.write(self._state)
    def write(self, val):
        with open(f"/sys/class/leds/{self._name}/brightness","w") as f:
            self._state = val
            f.write(f"{self._state}")
    def on(self):
        self.write(1)
    def off(self):
        self.write(0)
    def toggle(self):
        self.write(1 - self._state)
    def blink(self):
        with open(f"/sys/class/leds/{self._name}/trigger","w") as f:
            f.write("timer")
    def stop(self):
        with open(f"/sys/class/leds/{self._name}/trigger","w") as f:
            f.write("none")
        self.off()
