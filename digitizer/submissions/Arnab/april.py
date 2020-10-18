import apriltag
import cv2 as cv
import numpy
import math
from ros import ros

class april(ros):
    def __init__(self):
        super(april, self).__init__()

    def perform(self):
    	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    	detector = apriltag.Detector()
    	result = detector.detect(gray)
    	print(len(result), result)
    	for r in result:
    		angle = points(r.center, r.corners)
    		print 'center', result[0].center
    		print 'angle', angle

def show():
	cv.imshow("img", img)
	cv.imwrite("img.jpg", img)
	cv.waitKey()

def points(center, corners):
	print center, corners

	cv.circle(img, (int(center[0]), int(center[1])), 1, (0, 0, 255), 2)

	for i, points in enumerate(corners):
		cv.putText(img, str(i + 1), (int(points[0]), int(points[1])),
				   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv.LINE_AA)
		# cv.circle(img, (int(points[0]), int(points[1])), 1 + 2 * i, (255, 0, 0), 2)

	x0, y0 = center[0], center[1]
	x1, y1 = corners[0][0], corners[0][1]
	x2, y2 = corners[1][0], corners[1][1]

	x_mid, y_mid = (x1 + x2)/2, (y1 + y2)/2
	angle = math.atan2(y0 - y_mid, x0 - x_mid) # * 180 / math.pi

	R = 50
	x, y = x0 - R * math.cos(angle), y0 -  R * math.sin(angle)
	cv.arrowedLine(img, (int(x0),int(y0)), (int(x),int(y)), (0,255,0), 2, 8,0,0.1)
	return angle



if __name__ == '__main__':
	img = cv.imread('april_tag_1.jpg', cv.IMREAD_COLOR)
	# img = img[650:700, 400:450]
	operation(img)
	show()
