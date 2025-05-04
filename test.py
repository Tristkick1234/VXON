import cv2
from ultralytics import YOLO
from itertools import chain
import os
import pigpio
from time import sleep
#from sshkeyboard import listen_keyboard, stop_listening
import threading
# Initialize pigpio and set up the servo
pi = pigpio.pi()
servo_pin = 17  # GPIO pin 17 (use BCM numbering)

# Define minimum and maximum pulse widths
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500

# Initialize the servo to 0 degrees
rot = 0

# Function to set the servo angle
def set_angle(angle):
    global rot
    pulse_width = int(MIN_PULSE_WIDTH + (angle / 180.0 * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)))
    pi.set_servo_pulsewidth(servo_pin, pulse_width)

# Function to handle continuous rotation
def rotate_servo(step):
    global rot
    while annotated_frame:
        rot = max(15, min(rot + step, 180))
        set_angle(rot)
        print(f"Angle set to: {rot}")
        sleep(0.09)  # Adjust the speed of rotation here



# Load the YOLOv8 model
model = YOLO("yolov5nu.pt")
# Open the video file

cap = cv2.VideoCapture(0)

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()



    if success:
        # Run YOLOv8 inference on the frame
        results = model.predict(frame, classes=0, conf=0.7, max_det=1,show_labels=False, show_conf=False, verbose=False)


        # Visualize the results on the frame
        try:
            annotated_frame = results[0].plot()
            coords = results[0].boxes.xyxy
            reg = coords.tolist()
            [objcord] = chain(reg)
            print(objcord)

            if objcord:
                objcenterx = int((objcord[0] + objcord[2]) // 2)
                objcentery = int((objcord[1] + objcord[3]) // 2)
                cv2.circle(annotated_frame, (objcenterx, objcentery), 5, (255, 255, 0), -1)
                print(f"Object detected at {coords}")

                centerx = int(cap.get(3) // 2)
                centery = int(cap.get(4) // 2)

                cv2.circle(annotated_frame, (centerx, centery), 5, (255, 255, 0), -1)

                x_offset = centerx - objcenterx
                print(f"The offset is {x_offset}")
                if x_offset != 0:
                    global rotate_thread
                    if x_offset >20:
                        #rotate positive
                        print("rotating positive")
                        rotate_thread = threading.Thread(target=rotate_servo, args=(1,))

                    elif x_offset<20:
                        #rotate negative
                        print("rotating negative")
                        rotate_thread = threading.Thread(target=rotate_servo, args=(-1,))
                    if rotate_thread:
                        rotate_thread.start()
                else:
                    rotate_thread.join()
        except:
            print("no object detected")
            os.system("clear")







        #meanx = avg(boxes[0] boxes[2])
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            pi.set_servo_pulsewidth(servo_pin, 0)  # Stop the servo
            pi.stop()
            print("Program exited cleanly")
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()



# Clean up
