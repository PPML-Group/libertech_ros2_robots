import mediapipe as mp
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import torch
import pytorch_kinematics as pk
import trimesh
import os

from ament_index_python.packages import get_package_share_directory
package_share_directory = get_package_share_directory("libertech_hand_driver")
MESH_PATH = os.path.join(package_share_directory, "meshes")
print(MESH_PATH)


class HandModelVision:
    def __init__(self,
                 hand_file,
                 static_image_mode=True,
                 mesh_path=None, 
                 device='cpu'):
        self.hand_file = hand_file
        self.is_static = static_image_mode
        
        self.hand_mp_sol = mp.solutions.hands
        self.hand_mp = self.hand_mp_sol.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=2,
            model_complexity=1, # 更准确
            min_detection_confidence=0.65,
            min_tracking_confidence=0.65);     
            # 创建了 Hands 类的实例，用于处理手部关键点检测。 
            # 'hands'对象将用于处理每一帧图像，检测手的关键点
        self.draw_mp = mp.solutions.drawing_utils
        # self.finger_index = [[0, 1, 2, 3, 4],
        #        [0, 5, 6, 7, 8],
        #        [0, 9, 10, 11, 12],
        #        [0, 13, 14, 15, 16],
        #        [0, 17, 18, 19, 20]]
        # self.finger_end_index = [4, 8, 12, 16, 20]
        
        self.finger_index = [
               [0, 5, 6, 7, 8],
               [0, 9, 10, 11, 12],
               [0, 13, 14, 15, 16],
               [0, 17, 18, 19, 20],
               [0, 1, 2, 3, 4],]
        self.finger_end_index = [8, 12, 16, 20, 4, ]
        
        self.fig = plt.figure(figsize=(10, 5))
        self.ax1 = self.fig.add_subplot(121, projection='3d')
        self.ax2 = self.fig.add_subplot(122, projection='3d')
        ##############################
        self.device = device
        # not build_serial_chain_from_urdf
        self.chain = pk.build_chain_from_urdf(
            open(hand_file).read()).to(dtype=torch.float, device=device)
        self.n_dofs = len(self.chain.get_joint_parameter_names())
        print(self.n_dofs)
        # print(self.chain.get_joint_parameter_names())

        self.mesh = {}
        
        def build_mesh_recurse(body):
            if (len(body.link.visuals) > 0):
                # print(body.link)
                # print(body.link.visuals)
                link_vertices = []
                link_faces = []
                n_link_vertices = 0
                for visual in body.link.visuals:
                    if visual.geom_type is None:
                        continue
                    else:
                        scale = torch.tensor(
                            [1, 1, 1], dtype=torch.float, device=device)
                        if visual.geom_type == "box":
                            link_mesh = trimesh.primitives.Box(
                                extents=2*visual.geom_param)
                        elif visual.geom_type == "capsule":
                            link_mesh = trimesh.primitives.Capsule(
                                radius=visual.geom_param[0], height=visual.geom_param[1]*2).apply_translation((0, 0, -visual.geom_param[1]))
                        else:
                            # print(visual.geom_param[0])
                            # print(mesh_path)
                            # link_mesh = trimesh.load_mesh(
                            #     os.path.join(mesh_path, visual.geom_param[0].split(":")[1]+".obj"), process=False)
                            # print(os.path.join(mesh_path, visual.geom_param[0]+".STL"))
                            # link_mesh = trimesh.load_mesh(
                            #     os.path.join(mesh_path, visual.geom_param[0]+".STL"), process=False)
                            path = visual.geom_param[0]
                            if path.startswith("file://"):
                                path = path[len("file://"):]
                            link_mesh = trimesh.load_mesh(path, process=False)
                            if visual.geom_param[1] is not None:
                                scale = (visual.geom_param[1]).to(dtype=torch.float, device=device)
                        vertices = torch.tensor(
                            link_mesh.vertices, dtype=torch.float, device=device)
                        faces = torch.tensor(
                            link_mesh.faces, dtype=torch.float, device=device)
                        pos = visual.offset.to(dtype=torch.float, device=device)
                        vertices = vertices * scale
                        vertices = pos.transform_points(vertices)
                        link_vertices.append(vertices)
                        link_faces.append(faces + n_link_vertices)
                        n_link_vertices += len(vertices)
                    link_vertices = torch.cat(link_vertices, dim=0)
                    link_faces = torch.cat(link_faces, dim=0)
                    self.mesh[body.link.name] = {'vertices': link_vertices,
                                                'faces': link_faces,
                                                }
            for children in body.children:
                build_mesh_recurse(children)
        # print(self.chain._root)
        build_mesh_recurse(self.chain._root)

        self.joints_names = []
        self.joints_lower = []
        self.joints_upper = []

        # def set_joint_range_recurse(body):
        #     if body.joint.joint_type != "fixed":
        #         print(body.joint)
        #         self.joints_names.append(body.joint.name)
        #         self.joints_lower.append(body.joint.range[0])
        #         self.joints_upper.append(body.joint.range[1])
        #     for children in body.children:
        #         set_joint_range_recurse(children)
        # set_joint_range_recurse(self.chain._root)
        # self.joints_lower = torch.stack(
        #     self.joints_lower).float().to(device)
        # self.joints_upper = torch.stack(
        #     self.joints_upper).float().to(device)

        self.hand_pose = None
        self.global_translation = None
        self.global_rotation = None
        self.current_status = None
        
    def pred_hand_pose_mediapipe(self, img: np.ndarray, draw=True):
        # 默认输入RGB
        results = self.hand_mp.process(img) 
        img_draw= img.copy()
        hand_positions = {'right': [], 'left': []}
        if results.multi_hand_world_landmarks:
            for ih, handedness in enumerate(results.multi_handedness):
                # print(handedness)
                joint_positions = np.empty((21, 3)) 
                for i in range(21):
                    joint_positions[i] = np.array([results.multi_hand_world_landmarks[ih].landmark[i].x,
                                                results.multi_hand_world_landmarks[ih].landmark[i].y,
                                                results.multi_hand_world_landmarks[ih].landmark[i].z])
                # https://github.com/google/mediapipe/issues/4785
                # hand_type = 'right' if handedness.classification[0].label == 'Right' else 'left'
                hand_type = 'right' if handedness.classification[0].label == 'Left' else 'left'                          
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
                
        if hand_positions['left']:
            self.ax1.clear()
            self.plot_hand_pose_3d(self.ax1, hand_positions['left'][0])
            # rot = np.array([[0,0,-1],[0,-1,0],[-1,0,0]]) # TODO
            # self.plot_hand_pose_3d(self.ax1, self.transform_hand_pose(hand_positions['left'][0],rot))
            
        if hand_positions['right']:
            self.ax2.clear()
            # self.plot_hand_pose_3d(self.ax2, hand_positions['right'][0])
            rot = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
            self.plot_hand_pose_3d(self.ax2, self.transform_hand_pose(hand_positions['right'][0],rot))
        
        self.ax1.set_title('Left Hand') 
        self.ax2.set_title('Right Hand')
        
        self.fig.suptitle('Hand Positions')
        plt.show(block=False)
    
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
        ax.legend(['Index', 'Middle', 'Ring', 'Little', 'Thumb', 'End Points'], loc='best')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
    
    #######################################################################33
        
    def get_trimesh_data(self, i):
        """
        Get full mesh
        
        Returns
        -------
        data: trimesh.Trimesh
        """
        data = trimesh.Trimesh()
        for link_name in self.mesh:
            v = self.current_status[link_name].transform_points(
                self.mesh[link_name]['vertices'])
            if len(v.shape) == 3:
                v = v[i]
            # v = v @ self.global_rotation[i].T + self.global_translation[i]
            v = v.detach().cpu()
            f = self.mesh[link_name]['faces'].detach().cpu()
            data += trimesh.Trimesh(vertices=v, faces=f)
        return data
    
    def set_parameters(self, hand_pose):
        """
        Set translation, rotation, and joint angles of grasps
        
        Parameters
        ----------
        hand_pose: (B, 3+6+`n_dofs`) torch.FloatTensor
            translation, rotation in rot6d, and joint angles
        """
        self.hand_pose = hand_pose
        if self.hand_pose.requires_grad:
            self.hand_pose.retain_grad()
        # self.global_translation = self.hand_pose[:, 0:3]
        # self.global_rotation = robust_compute_rotation_matrix_from_ortho6d(
        #     self.hand_pose[:, 3:9])
        # self.current_status = self.chain.forward_kinematics(
        #     self.hand_pose[:, 9:])
        self.current_status = self.chain.forward_kinematics(
            self.hand_pose[:, :])
    
# %% main  
if __name__ == '__main__':
    hand_file = '/home/ccs/ros2/libertech_ws/src/libertech_ros2_robots/libertech_hand_description/urdf/hand_right.xacro.urdf'
    
    hand_vision_tool = HandModelVision(hand_file, 
                                       filter_k=0.5,
                                       mesh_path=MESH_PATH)
    
    img = cv2.imread("./image.jpg")
    # 默认输入RGB
    cv2.imshow("img", img)
    cv2.waitKey(1000)
    
    hand_positions, img = hand_vision_tool.pred_hand_pose_mediapipe(img[...,::-1])
    # print(hand_positions)
    
    cv2.imshow("img_res", img[...,::-1])
    cv2.waitKey(1000)
    
    # print(hand_vision_tool.extract_endpoints(hand_positions['right'][0]))
    
    hand_vision_tool.plot_hands(hand_positions)
    
    
    