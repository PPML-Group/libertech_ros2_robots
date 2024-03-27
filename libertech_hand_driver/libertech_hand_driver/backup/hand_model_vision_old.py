import mediapipe as mp
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class HandModelVision:
    def __init__(self,
                 hand_file,
                 filter_k,
                 static_image_mode=True):
        self.hand_file = hand_file
        self.filter_k = filter_k
        
        self.hand_mp_sol = mp.solutions.hands
        self.hand_mp = self.hand_mp_sol.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=1,
            model_complexity=0, # 更准确
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5);     
            # 创建了 Hands 类的实例，用于处理手部关键点检测。 
            # 'hands'对象将用于处理每一帧图像，检测手的关键点
        self.draw_mp = mp.solutions.drawing_utils
        
        self.finger_index = [[0, 1, 2, 3, 4],
               [0, 5, 6, 7, 8],
               [0, 9, 10, 11, 12],
               [0, 13, 14, 15, 16],
               [0, 17, 18, 19, 20]]
        self.finger_end_index = [4, 8, 12, 16, 20]
        
    def pred_hand_pose_mediapipe(self, img: np.ndarray, draw=True):
        # 默认输入RGB
        results = self.hand_mp.process(img) 
        img_draw= img.copy()
        hand_positions = {'right': [], 'left': []}
        if results.multi_hand_world_landmarks:
            for ih, handedness in enumerate(results.multi_handedness):
                print(handedness)
                joint_positions = np.empty((21, 3)) 
                for i in range(21):
                    joint_positions[i] = np.array([results.multi_hand_world_landmarks[ih].landmark[i].x,
                                                results.multi_hand_world_landmarks[ih].landmark[i].y,
                                                results.multi_hand_world_landmarks[ih].landmark[i].z])
                hand_type = 'right' if handedness.classification[0].label == 'Right' else 'left'
                hand_positions[hand_type].append(joint_positions)
                if draw:
                    # 在img上添加手的图像 handLms 节点坐标  mpHands.HAND_CONNECTIONS 连接线
                    self.draw_mp.draw_landmarks(img_draw, 
                                                results.multi_hand_landmarks[ih], 
                                                self.hand_mp_sol.HAND_CONNECTIONS)  
        # 低通滤波 写在主函数
        # joint_positions_data_filter = self.filter_k * joint_positions + (1-self.filter_k)*joint_positions_data_filter  
        
        return hand_positions, img_draw
    
    def transform_hand_pose(self, hand_positions: np.ndarray, rot: np.ndarray) -> np.ndarray:
        # 将每个手部节点的位置向量与旋转矩阵相乘
        transformed_positions = np.dot(rot, hand_positions.T).T
        return transformed_positions    
    
    def extract_endpoints(self, hand_positions):
        finger_endpoints = []
        for finger in self.finger_index:
            finger_endpoint = hand_positions[finger[-1]]
            finger_endpoints.append(finger_endpoint)
        return finger_endpoints


    def plot_hands(self, hand_positions):
        fig = plt.figure(figsize=(10, 5))
        ax1 = fig.add_subplot(121, projection='3d')
        ax2 = fig.add_subplot(122, projection='3d')
        
        rot = np.array([[0,0,-1],[0,-1,0],[-1,0,0]])
        # self.plot_hand_pose_3d(ax1, self.transform_hand_pose(hand_positions['right'][0],rot))
        # self.plot_hand_pose_3d(ax2, self.transform_hand_pose(hand_positions['left'][0],rot))
        if hand_positions['right']:
            self.plot_hand_pose_3d(ax1, hand_positions['right'][0])
        if  hand_positions['left']:
            self.plot_hand_pose_3d(ax2, hand_positions['left'][0])
        
        ax1.set_title('Right Hand')
        ax2.set_title('Left Hand')
        fig.suptitle('Hand Positions')
        plt.show()
    
    def plot_hand_pose_3d(self, ax: Axes3D, hand_positions):
        # 绘制手指关节点
        for finger in self.finger_index:
            ax.plot(hand_positions[finger, 0], 
                    hand_positions[finger, 1], 
                    hand_positions[finger, 2], 
                    marker='o', linestyle='-')
        # 加粗显示手指末端节点
        ax.scatter(hand_positions[self.finger_end_index, 0], 
                   hand_positions[self.finger_end_index, 1], 
                   hand_positions[self.finger_end_index, 2], 
                   c='r', s=100)
        # ax.set_box_aspect([1,1,1])
        ax.set_aspect('equal')
        ax.legend(['Thumb', 'Index', 'Middle', 'Ring', 'Little', 'End Points'], loc='best')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    
# %% main  
if __name__ == '__main__':
    hand_file = '/home/ccs/ros2/libertech_ws/src/libertech_ros2_robots/libertech_hand_description/urdf/hand_right.xacro.urdf'
    
    hand_vision_tool = HandModelVision(hand_file, filter_k=0.5)
    
    img = cv2.imread("./image.jpeg")
    # 默认输入RGB
    cv2.imshow("img", img)
    cv2.waitKey(1000)
    
    hand_positions, img = hand_vision_tool.pred_hand_pose_mediapipe(img[...,::-1])
    print(hand_positions)
    
    cv2.imshow("img_res", img[...,::-1])
    cv2.waitKey(1000)
    
    # print(hand_vision_tool.extract_endpoints(hand_positions['right'][0]))
    
    hand_vision_tool.plot_hands(hand_positions)
    
    
    