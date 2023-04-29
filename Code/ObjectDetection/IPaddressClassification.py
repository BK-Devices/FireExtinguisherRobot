import cv2 
import urllib.request 
import numpy as np



url = 'http://192.168.43.16/cam-hi.jpg'

winName = 'ESP32 CAMERA'
cv2.namedWindow(winName,cv2.WINDOW_AUTOSIZE)


classNames = []
classFile = 'coco.names'
with open(classFile,'r') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
#net.setInputSize(480,480)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

while(1):
    imgResponse = urllib.request.urlopen (url) 
    imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
    img = cv2.imdecode (imgNp,-1) 

    #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) 
    #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)# vertical
   

    

    classIds, confs, bbox = net.detect(img,confThreshold=0.5)
    print(classIds,bbox)

    if len(classIds) != 0:
        for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
            cv2.rectangle(img,box,color=(0,255,0),thickness = 3) 
            cv2.putText(img, classNames[classId-1], (box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0),2)


    cv2.imshow(winName,img)

    
    tecla = cv2.waitKey(5) & 0xFF
    if tecla == 27:
        break
cv2.destroyAllWindows()
