import cv2

print("cv2:", cv2.__version__)
print("has aruco:", hasattr(cv2, "aruco"))
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
print("DICT_4X4_50 ArucoDetector OK")
