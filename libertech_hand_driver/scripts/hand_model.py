import numpy as np
import time
from libertech_hand_driver.hand_model_kinematics import HandModelKinematics, logger
from libertech_hand_driver.hand_model_vision import HandModelVision
from cprint import cprint
import cv2
import torch

import plotly.graph_objects as go
import matplotlib.pyplot as plt
import trimesh

if __name__ == '__main__':
    hand_file = '/home/ccs/ros2/libertech_ws/src/libertech_ros2_robots/libertech_hand_description/urdf/hand_right.xacro.urdf'
    
    hand = HandModelKinematics(hand_file)
        
    logger.info('*' * 15 + '测试食指 (中指)关节-驱动正逆运动学' + '*' * 15)
    q1_sequence = np.arange(-0.4, 0.4, 0.05)
    q2_sequence = np.arange(0, 1.75, 0.05)
    n_error = 0
    t_ik_sum = 0
    t_fk_sum = 0
    pos = np.array([0.0, 0.0, 0.4])
    for q1 in q1_sequence:
        for q2 in q2_sequence:
            pos[0] = q1
            pos[1] = q2
            # print(f"设定的关节角为: {pos}")
            
            ts = time.time()
            d = hand.ik_q2d(pos, "index")
            t_ik_sum += time.time()-ts
            
            ts = time.time()
            pos_fk = hand.fk_d2q(d, "index")
            t_fk_sum += time.time()-ts
            
            # print(f"pos误差为: {pos - pos_fk[:3]}")
            if np.linalg.norm(pos - pos_fk[:3]) > 0.005:
                n_error += 1
          
    logger.warning(f"求解错误次数为: {n_error}")
    logger.info(f"食指 (中指)关节-驱动逆运动学平均用时: {t_ik_sum/(np.size(q1_sequence)*np.size(q2_sequence))}")
    logger.info(f"食指 (中指)关节-驱动正运动学平均用时: {t_fk_sum/(np.size(q1_sequence)*np.size(q2_sequence))}")
    
    logger.info('*' * 15 + '测试无名指 (小拇指)关节-驱动正逆运动学' + '*' * 15)
    q1_sequence = np.arange(-0.4, 0.4, 0.05)
    q2_sequence = np.arange(0, 1.75, 0.05)
    n_error = 0
    t_ik_sum = 0
    t_fk_sum = 0
    pos = np.array([0.0, 0.0])
    for q1 in q1_sequence:
        for q2 in q2_sequence:
            pos[0] = q1
            pos[1] = q2
            
            ts = time.time()
            d = hand.ik_q2d(pos, "fourth")
            t_ik_sum += time.time()-ts
            
            ts = time.time()
            pos_fk = hand.fk_d2q(d, "fourth")
            t_fk_sum += time.time()-ts
            
            # print(f"pos误差为: {pos - pos_fk[:2]}")
            if np.linalg.norm(pos - pos_fk[:2]) > 0.005:
                cprint.err(f"设定的关节角为: {pos},pos误差为: {pos - pos_fk[:2]}")
                n_error += 1
                
    logger.warning(f"求解错误次数为: {n_error}")
    logger.info(f"无名指 (小拇指)关节-驱动逆运动学平均用时: {t_ik_sum/(np.size(q1_sequence)*np.size(q2_sequence))}")
    logger.info(f"无名指 (小拇指)关节-驱动正运动学平均用时: {t_fk_sum/(np.size(q1_sequence)*np.size(q2_sequence))}")
    
    
    logger.info('*' * 15 + '测试关节-末端正逆运动学' + '*' * 15)
    q1_sequence = np.arange(-0.4, 0.4, 0.5)
    q2_sequence = np.arange(0, 1.75, 0.5)
    q3_sequence = np.arange(0, 1.70, 0.5)
    size = np.size(q1_sequence)*np.size(q2_sequence)*np.size(q3_sequence)
    n_error = 0
    t_ik_sum = 0
    t_fk_sum = 0
    pos = np.array([0.0, 0.0, 0.4])
    for q1 in q1_sequence:
        for q2 in q2_sequence:
            for q3 in q3_sequence:
                pos[0] = q1
                pos[1] = q2
                pos[2] = q3
                
                ts = time.time()
                fk = hand.fk_q2e(pos, "index")
                pe = fk.t
                # print(fk)
                # logger.info(np.round(p[1],decimals=6))
                # exit()
                t_fk_sum += time.time()-ts
                
                ts = time.time()
                pos_ik = hand.ik_e2q(pe, "index")
                t_ik_sum += time.time()-ts
                
                # print(f"pos误差为: {pos - pos_fk[:3]}")
                # print(f'pos: {pos}')
                # print(f'pos_ik: {pos_ik}')
                if np.linalg.norm(pos - pos_ik) > 0.005:
                    n_error += 1
    logger.warning(f"求解错误次数为: {n_error}")
    logger.info(f"关节-末端逆运动学平均用时: {t_ik_sum/size}")
    logger.info(f"关节-末端正运动学平均用时: {t_fk_sum/size}")
    
    
    logger.info('*' * 15 + 'testing retarget' + '*' * 15)
    hand_vision_tool = HandModelVision(hand_file, 
                                       static_image_mode=False)
    
    img = cv2.imread("./libertech_hand_driver/image_real_cap.png")
    # 默认输入RGB
    cv2.imshow("img", img)
    cv2.waitKey(500)
    
    hand_positions, img = hand_vision_tool.pred_hand_pose_mediapipe(img[...,::-1])
    print(hand_positions)
    
    cv2.imshow("img_res", img[...,::-1])
    cv2.waitKey(500)
    
    try:
        if hand_positions['right']:
            rot = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
            hand_positions_rot = hand_vision_tool.transform_hand_pose(hand_positions['right'][0],rot)
            ph = hand_vision_tool.extract_endpoints(hand_positions_rot)
            
            # print(ph)
            # exit()
    except:
        exit()
    
    hand_vision_tool.plot_hands(hand_positions)
    
    ts = time.time()
    q = hand.ik_e2q_retarget(ph)
    logger.info(f"retarget用时为: {1000*(time.time()-ts)} ms, q为: {q}")
    
    fk_all = hand.fk_q2e_all(q)
    pa = [fk.t for fk in fk_all]
    logger.info(f"ret_h: {hand.compute_directions_distances(ph, is_unit=True)}")
    logger.info(f"ret_a: {hand.compute_directions_distances(pa, is_unit=True)}")
    
    # ret_h = hand.compute_directions_distances(ph, is_unit=True)
    # logger.info(f"ret_a: {ret_h}")
    # fd = ret_h['finger_to_thumb_distances']
    # sd = np.ones_like(ret_h['finger_to_thumb_distances'])
    # fd[ret_h['finger_to_thumb_distances'] < 0.02] = 0.001
    # sd[ret_h['finger_to_thumb_distances'] < 0.02] = 200
    # logger.info(f"fd: {fd}")
    # logger.info(f"sd: {sd}")
    # logger.info(f"test d: {fd[:, np.newaxis] * ret_h['finger_to_thumb_directions']}")
    # logger.info(f"test d: {sd[:, np.newaxis] * (fd[:, np.newaxis] * ret_h['finger_to_thumb_directions'])**2}")
    
    
    logger.info('*' * 15 + '测试3D Mesh Plot' + '*' * 15)
    # q = np.zeros(13)
    # q[10] = 1.
    # q[7] = 1.
    # q[9] = 1.
    # q[2] = 1.
    # q[5] = 1.
    # q[11] = 1.
    # q[12] = 1.
    qa = hand.fk_q2q_all(q)
                   
    hand_pose = torch.tensor(qa, dtype=torch.float, device="cpu").unsqueeze(0)
    # hand_pose = torch.zeros(1, 20, dtype=torch.float, device="cpu")
    hand_vision_tool.set_parameters(hand_pose)
    hand_mesh = hand_vision_tool.get_trimesh_data(0)
    # hand_mesh.show()
    
    vertices = hand_mesh.vertices
    faces = hand_mesh.faces
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=faces, cmap='viridis', edgecolor='none')
    # plt.show()
    
    fig = go.Figure(data=[
        go.Mesh3d(
            x=vertices[:, 0],
            y=vertices[:, 1],
            z=vertices[:, 2],
            i=faces[:, 0],
            j=faces[:, 1],
            k=faces[:, 2],
            color='lightpink',
            opacity=0.50
        )
    ])
    fig.show()
    plt.show()
    
    
    # cap = cv2.VideoCapture(0)
    # while 1:
    #     success, img = cap.read()
    #     imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
    #     hand_positions, img = hand_vision_tool.pred_hand_pose_mediapipe(img[...,::-1])
    #     # print(hand_positions)
        
    #     cv2.imshow("img cap", img[...,::-1])
    #     cv2.waitKey(1)