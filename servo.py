from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, Servo, Buzzer
import time

Device.pin_factory = PiGPIOFactory()

horizontal = Servo(pin=17, min_pulse_width=400/1000000, max_pulse_width=2400/1000000)
vertical = Servo(pin=27, min_pulse_width=400/1000000, max_pulse_width=2400/1000000)
bz = Buzzer(22)

horizontal.value = 0
vertical.value = 0
bz.on()
time.sleep(1)
bz.off()
horizontal.value = 1
vertical.value = 1
time.sleep(1)
horizontal.value = -1
vertical.value = -1
time.sleep(1)
horizontal.value = 0
vertical.value = 0
