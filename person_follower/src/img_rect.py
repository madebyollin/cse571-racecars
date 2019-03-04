import cv2

fname = '001.png'
img = cv2.imread(fname)
cv2.rectangle(img, (212,317), (290,436), (0,255,0), 4)
font = cv2.FONT_HERSHEY_SIMPLEX
text = '001'
cv2.putText(img, text, (212, 310), font, 2, (0,0,255), 1)
cv2.imwrite('001_new.jpg', img)
