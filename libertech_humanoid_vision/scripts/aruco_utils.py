import numpy as np
import cv2
import sys

class ArucoDetector:
    def __init__(self, 
                 id,
                 aruco_dict,
                 camera_matrix,
                 dist_coeffs,
                 aruco_size
                 ):
        self.id = id
        self.camera_matrix = np.array(camera_matrix).reshape((3, 3))
        self.dist_coeffs = np.asarray(dist_coeffs)
        self.aruco_size = aruco_size
        
        dict = getattr(cv2.aruco, aruco_dict, None)
        # print(dict)
        if dict is None:
            print("Invalid ArUco dictionary")
            sys.exit(1)
            
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict)
        self.parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        
    def get_feature(self, img):
        # 4.7.x修改了API
        # dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        # parameters =  cv.aruco.DetectorParameters()
        # detector = cv.aruco.ArucoDetector(dictionary, parameters)
        # frame = cv.imread(...)
        # markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
        
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(img)
        # cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
        # print(f'markerCorners: {markerCorners}')
        # img = self.draw_aruco(markerCorners, markerIds, rejectedCandidates, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))   
        s = None
        if markerIds is not None:   
            for (markerCorner, markerID) in zip(markerCorners, markerIds):
                if markerID == self.id:
                    s = markerCorners[0][0]  # list np 1 4 2
        else:
            s = None
        return s
    
    def draw_feature(self, img, s):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  ## 
        s_list = [s]
        for si in s_list:
            (topLeft, topRight, bottomRight, bottomLeft) = si
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(img, topLeft, topRight, (255, 0, 0), 2)
            cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(img, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(img, bottomLeft, topLeft, (255, 255, 0), 2)
            
            cv2.putText(img, str(self.id),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        return img
    
    
    def detect_and_estimate_pose(self, image):
        # 检测 ArUco 码
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(image)

        if ids is None:
            print("No ArUco markers found")
            return None

        # 估计 ArUco 码的姿态
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_matrix, self.dist_coeffs)
        # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, self.camera_matrix, np.array([0,0,0,0,0]))
        # 获取特定ID的姿态信息并绘制
        for i in range(len(ids)):
            if ids[i] == self.id:
                # 绘制坐标系
                cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.aruco_size)

                # 绘制角点
                cv2.aruco.drawDetectedMarkers(image, corners)

                return rvecs[i], tvecs[i], image

        print("ArUco marker with specified ID not found")
        return None
    
    