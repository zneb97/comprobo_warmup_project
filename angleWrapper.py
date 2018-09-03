import math

pi = math.pi

def radianDifference(ang1, ang2):
	"""
	Takes in two angles in radians and calculates the angle 
	between them such that the return is between -pi and pi
	"""

	#Get angle reduced to between 
	if ang1 != 0:
		ang1 = 2*pi-(2*pi*math.ceil(ang1/2*pi)-ang1)
		if ang1 == pi:
			ang1 =0

	if ang2 != 0:
		ang2 = pi-(pi*math.ceil(ang2/pi)-ang2)
		if ang2 == pi:
			ang2 =0


	print(ang1, ang2)
	return 0

if __name__ == "__main__":
	radianDifference(3.1, 3.2)