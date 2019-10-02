from oculusTracking import CV1Sensor
import cv2
import time
import numpy as np
import glob
import argparse
from tqdm import tqdm
import json
import os


def saveCheckerboard(dim=(9,6), boxSize=50):
	img = np.zeros(shape=((dim[0]+1)*boxSize, (dim[1]+1)*boxSize), dtype=np.uint8)
	for row in range(dim[0]):
		for col in range(dim[1]):
			if (row + col) % 2 == 0:
				img[row*boxSize:(row+1)*boxSize, col*boxSize:(col+1)*boxSize] = 255

	cv2.imwrite(f'checkerboard{dim[0]}x{dim[1]}.png', img)


parser = argparse.ArgumentParser()
# parser.add_argument('--save', action='store_true', help='open video stream to save calibration images')
parser.add_argument('--calculate', action='store_true', help='load calibration images and calculate calibration data')
parser.add_argument('--onlyconnected', action='store_true', help='only calculate calibration data for the connected sensor')
parser.add_argument('--view', action='store_true', help='display undistorted stream using calibration data')
args = parser.parse_args()



if args.calculate:

	boardPoints = np.zeros(shape=(6*9,3), dtype=np.float32)
	boardPoints[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1, 2)
	boardPoints *= 0.0177
	
	if not args.onlyconnected:
		basePaths = glob.glob('calibration/*')
	else:
		sensor = CV1Sensor()
		basePaths = [f'calibration/{sensor.serialNumber()}']

	for baseCalPath in basePaths:
		print(baseCalPath)

		objPoints = []
		imgPoints = []

		calImages = glob.glob(f'{baseCalPath}/img_*.png')
		print('loading calibration images')
		for imgName in tqdm(calImages):
			img = cv2.imread(imgName)

			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			found, corners = cv2.findChessboardCorners(gray, (9,6), None)
			if found:
				subPixCorners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001))
				objPoints.append(boardPoints)
				imgPoints.append(subPixCorners)
				cv2.drawChessboardCorners(img, (9,6), subPixCorners, found)
			else:
				os.remove(imgName)

			cv2.imshow(imgName, img)
			if cv2.waitKey(1) == 27:
				exit()
			cv2.destroyAllWindows()

		print('calculating calibration data')
		ret, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, gray.shape[::-1], None, None)
		print(ret)
		
		h, w = img.shape[:2]
		newMatrix, roi = cv2.getOptimalNewCameraMatrix(matrix, dist, (w,h), 1, (w,h))

		print('saving data')
		with open(f'{baseCalPath}/data.json', 'w') as outfile:
			data = {
				'matrix': matrix.tolist(),
				'dist': dist.tolist(),
				'newMatrix': newMatrix.tolist(),
			}
			json.dump(data, outfile, indent=' '*4)

else:
	
	sensor = CV1Sensor()
	sensor.start()

	time.sleep(1)
	sensor.sensor.setGain(128)
	sensor.sensor.setCoarseExposureTime(800)

	baseCalPath = f'calibration/{sensor.serialNumber()}'
	if not os.path.exists(baseCalPath):
		os.makedirs(baseCalPath)
	print(baseCalPath)

	calImages = glob.glob(f'{baseCalPath}/img_*.png')

	if not args.view:
		startingNumber = 1 if len(calImages) == 0 else (max(map(lambda x: int(x.split('.')[0].split('_')[1]), calImages)) + 1)
		print(f'next image: {startingNumber}')
	else:
		with open(f'{baseCalPath}/data.json', 'r') as infile:
			data = json.load(infile)
			matrix = np.array(data['matrix'], dtype=np.float32)
			dist = np.array(data['dist'], dtype=np.float32)
			# print(data['dist'])
			# newMatrix = np.array(data['newMatrix'], dtype=np.float32)
			newMatrix, roi = cv2.getOptimalNewCameraMatrix(matrix, dist, (1280,810), 1, (1280,810))
			# fx, fy, cx, cy, k = sensor.calData
			# matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
			# dist = np.array([k[0], k[1], k[2], k[3], 0])
			# newMatrix, roi = cv2.getOptimalNewCameraMatrix(matrix, dist, (1280,960), 1, (1280,960))
			mapx, mapy = cv2.initUndistortRectifyMap(matrix, dist, None, newMatrix, (1280,810), 5)
			# print(k, roi)
	while True:
		if sensor.hasNewFrame():
			frameGray = sensor.latestFrame()[150:, :]
			# frameColor = cv2.cvtColor(frameGray, cv2.COLOR_GRAY2BGR)
			if args.view:
				frameGray = cv2.remap(frameGray, mapx, mapy, cv2.INTER_LINEAR)
				# frameGray = cv2.undistort(frameGray, matrix, dist, None, newMatrix)
			
			cv2.imshow('frame', frameGray)
			key = cv2.waitKey(1)
			if key == 27:
				exit()
			elif key == 32:
				imgName = f'{baseCalPath}/img_{startingNumber}.png'
				print(f'saving to {imgName}')
				cv2.imwrite(imgName, frameGray)
				startingNumber += 1


	sensor.stop()