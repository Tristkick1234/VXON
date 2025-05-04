import cv2
import numpy as np
from ultralytics import YOLO


# cap = cv2.VideoCapture(0)
#
# # fcascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
# model = YOLO("yolov8n.pt")
#
# while True:
#
#
#     ret, frame = cap.read()
#
# #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# #     faces = fcascade.detectMultiScale(gray, 1.05, 5)
# #     for (x, y, w, h) in faces:
# #         cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
#
#     # cv2.imwrite('mephoto.png',frame)
#     img = frame
#     #img might need to be png
#     results = model.predict(img)
#
#     # img = cv2.imread('meimg.png')
#     cords = [r.boxes.xyxy.tolist() for r in results]
#
#     [cords] = cords
#     cords = [[round(i) for i in x] for x in cords]
#
#
#     for i in range(len(cords)):
#         fx=cords[i][0]
#         fy=cords[i][1]
#         gx=cords[i][2]
#         gy=cords[i][3]
#         image = cv2.rectangle(img,(fx,fy),(gx,gy),(255,0,0),2)
#
#     cv2.imshow('Object detection in images',img)
#
#     if cv2.waitKey(1) == ord('e'):
#         break
#
# cap.release()
# cv2.destroyAllWindows()





# #img will be frame that is stopped


        

# # cv2.waitKey(0)
# # cv2.destroyAllWindows()

  
    
   


    



# #use the boxes position from the model.predict)
# #use opencv and do cv2. rectangle at that location

import cv2
from ultralytics import YOLO
from itertools import chain
import os

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
        results = model.predict(frame, classes=0, conf=0.7, max_det=1,show_labels=False, show_conf=False,verbose=False)


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

                c
                v2.circle(annotated_frame, (centerx, centery), 5, (255, 255, 0), -1)

                x_offset = centerx - objcenterx
                print(f"The offset is {x_offset}")
                if x_offset != 0:
                    if x_offset >0:
                        #rotate positive
                        print("rotating positive")
                    if x_offset<0:
                        #rotate negative
                        print("rotating negative")
        except:
            print("no object detected")
            os.system('cls')






        #meanx = avg(boxes[0] boxes[2])
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()