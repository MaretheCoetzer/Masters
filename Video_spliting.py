import cv2

video='TwoD_SS_78' 
capture = cv2.VideoCapture('C:/Users/Marethe/Downloads/'+video+'.mp4')
 
frameNr = 0
 
while (True):
 
    success, frame = capture.read()
 
    if success:
        cv2.imwrite(f'C:/Users/Marethe/Documents/GitHub/Masters/image_sorting/'+video+'_'+str(frameNr)+'.jpg', frame)
 
    else:
        break
 
    frameNr = frameNr+10
 
capture.release()