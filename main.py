import object_detection
import time
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, Servo, Buzzer
from simple_pid import PID

def inside(center, detection_box):
    in_x = (detection_box.origin_x < center[0] and (detection_box.origin_x + detection_box.width) > center[0])
    in_y = (detection_box.origin_y < center[1] and (detection_box.origin_y + detection_box.height) > center[1])
    return in_x and in_y

hFoV = 66.2
vFoV = 48.8

Device.pin_factory = PiGPIOFactory()

horizontal_servo = Servo(pin=17, min_pulse_width=400/1000000, max_pulse_width=2400/1000000)
vertical_servo = Servo(pin=27, min_pulse_width=400/1000000, max_pulse_width=2400/1000000)
bz = Buzzer(22)

horizontal_pid = PID(0.5, 0.25, 0, setpoint=0)
horizontal_pid.output_limits = (-1, 1)
vertical_pid = PID(0.5, 0.25, 0, setpoint=0)
vertical_pid.output_limits = (-1, 1)

horizontal_servo.value = 0
vertical_servo.value = 0
bz.off()

detector = object_detection.Detector(headless=False)

n=1000
start = time.time()

for i in range(n):
    detections = detector.capture_and_detect()
    if len(detections) > 0:
        bbox = detections[0].bounding_box
        
        # Horizontal
        x_mid_box = bbox.origin_x + bbox.width/2
        x_mid_cam = detector.x_mid
        y_mid_box = bbox.origin_y + bbox.height/2
        y_mid_cam = detector.y_mid
        e_x = (x_mid_box - x_mid_cam)/(x_mid_box*2) * hFoV/180
        e_y = (y_mid_box - y_mid_cam)/(y_mid_box*2) * vFoV/180
        horizontal_servo.value = horizontal_pid(e_x)
        vertical_servo.value = vertical_pid(e_y)
        
        if inside((detector.x_mid, detector.y_mid), bbox):
            # bz.beep()
            print("In")
bz.off()
stop = time.time()
print(f"{(stop-start)/n} seconds/capture")
if not detector.headless:
    time.sleep(10)