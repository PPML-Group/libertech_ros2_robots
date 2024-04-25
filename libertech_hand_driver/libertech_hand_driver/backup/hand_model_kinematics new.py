from roboticstoolbox.robot.ERobot import ERobot
import numpy as np
from spatialmath import SO3, SE3, UnitQuaternion
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import math
import time
from sko.PSO import PSO
# from pyswarm import pso
from scipy.optimize import fsolve, minimize
import urdf_parser_py.urdf as urdf
from math import cos, sin

from libertech_hand_driver.hand_cfg import *
from libertech_hand_driver.logger import logger

np.set_printoptions(precision=3, suppress=True)

class HandModelRTB(ERobot):
    def __init__(self,robot_path):
        links, name, urdf_string, urdf_filepath = self.URDF_read(robot_path)
        super().__init__(
            links,
            name=name.upper(),
            manufacturer="BIT-WGC",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
        with open(robot_path, 'rb') as file:
            urdf_content = file.read()

        self.urdf_ = urdf.Robot.from_xml_string(urdf_content)
        # 获取关节名称
        self.joint_names_ = []
        for joint in self.urdf_.joints:
            self.joint_names_.append(joint.name)

        
class HandModelKinematics:
    # %% 一些配置参数
    finger_mapping = {
        "index": 1,
        "middle": 2,
        "fourth": 3,
        "little": 4,
        "thumb": 5,
    }
    finger_driver_num = {
        "index": 3,
        "middle": 3,
        "fourth": 2,
        "little": 2,
        "thumb": 3,
    }
    finger_joint_index = {
        "index": 0,
        "middle": 3,
        "fourth": 6,
        "little": 8,
        "thumb": 10,
    }
    def __init__(self,
                 hand_file,
                 display=True):
        self.model_rtb_ = HandModelRTB(hand_file)
        self.qdim_ = self.model_rtb_.n
        self.qlim_ = self.model_rtb_.qlim
        self.qlim_act_ = []
        self.qname_tmp = self.model_rtb_.joint_names_[2:]
        self.qname_ = []
        self.qname_act_ = []
        for i in range(5):
            self.qlim_act_.append(self.qlim_[:, 4*i:4*i+list(self.finger_driver_num.items())[i][1]])
            self.qname_.extend(self.qname_tmp[5*i:5*i+4]) # end多了一个
            self.qname_act_.extend(self.qname_tmp[5*i:5*i+list(self.finger_driver_num.items())[i][1]]) # end多了一个
        self.qlim_act_ = np.hstack(self.qlim_act_)
        
        self.display_ = display
        if display:
            print(self.model_rtb_)
            logger.info(f"qlim: {self.qlim_}")
            logger.info(f"qlim_act: {self.qlim_act_}")
            logger.info(f"qname_: {self.qname_}")
            logger.info(f"qname_act_: {self.qname_act_}")
            # import swift
            # env = swift.Swift()
            # env.launch(realtime=True)
            # q = np.zeros((100,20))
            # self.model_rtb.plot(q, backend='pyplot', block=True, jointaxes=False, )
            # env.add(self.model_rtb)
            # plt.show()
        
        if self.model_rtb_.name.find("RIGHT") != -1:
            self.hand_dir_ = 1
            self.hand_name_ = "right"
            logger.info("Using right hand")
        elif self.model_rtb_.name.find("LEFT") != -1:
            self.hand_dir_ = -1
            self.hand_name_ = "left"
            logger.info("Using left hand")
        else:
            logger.error(f"未指定左右手, urdf name is {self.model_rtb_.name}")
            exit()
        
        # finger_name = "thumb"
        # finger_number = self.get_finger_number(finger_name)
        # logger.info(f"手指名字为'{finger_name}'，对应的编号为'{finger_number}'")
        
    def get_finger_number(self, finger_name: str):
        return self.finger_mapping.get(finger_name.lower(), "Invalid finger name")
    
    def get_module_config(self, name: str):
        return module_config[name]
            
    # %% 用于计算四杆机构
    def four_bar_forward(self, q, A, B, C, D, degree1_init, degree2_init):
        '''求解四杆机构角度关系,例如从PIP推导DIP
        '''    
        L1=math.sqrt(A**2+C**2-2*A*C*math.cos(q+degree1_init))
        parameter1=(C-A*math.cos(q+degree1_init))/L1
        parameter2=(L1**2+D**2-B**2)/(2*L1*D)
        parameter1 = max(min(parameter1, 1), -1)
        parameter2 = max(min(parameter2, 1), -1)
        return degree2_init-(math.acos(parameter2)-math.acos(parameter1))

        
    def four_bar_backward(self, q, A, B, C, D, degree1_init, degree2_init):
        L2=math.sqrt(C**2+D**2-2*C*D*math.cos(degree2_init-q))
        parameter1=math.sin(degree2_init-q)*D/L2
        parameter2=(L2^2+A^2-B^2)/(2*L2*A)
        parameter1 = max(min(parameter1, 1), -1)
        parameter2 = max(min(parameter2, 1), -1)
        return math.acos(parameter2)-math.asin(parameter1)-degree1_init
    
    def four_bar_pip2dip(self, q: float, name: str):
        '''所有手指的dip都与pip耦合

        :param q: _description_
        :type q: float
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        cfg = self.get_module_config(name)
        return self.four_bar_forward(q, cfg["A"],cfg["B"],cfg["C"],cfg["D"],cfg["degree1_init"],cfg["degree2_init"])
        
    def four_bar_dip2pip(self, q: float, name: str):
        cfg = self.get_module_config(name)
        return self.four_bar_backward(q, cfg["A"],cfg["B"],cfg["C"],cfg["D"],cfg["degree1_init"],cfg["degree2_init"])
        
    def four_bar_mcp2pip(self, q: float, name: str):
        assert (name=="fourth" or name=="little")
        cfg = self.get_module_config(name)
        return self.four_bar_forward(q, cfg["Am"],cfg["Bm"],cfg["Cm"],cfg["Dm"],cfg["degree1m_init"],cfg["degree2m_init"])
        
    def four_bar_pip2mcp(self, q: float, name: str):
        assert (name=="fourth" or name=="little")
        cfg = self.get_module_config(name)
        return self.four_bar_backward(q, cfg["Am"],cfg["Bm"],cfg["Cm"],cfg["Dm"],cfg["degree1m_init"],cfg["degree2m_init"])

    # %% 用于计算关节空间与工作空间的运动学映射（考虑关节耦合）
    def fk_q2e(self, q: np.ndarray, fg_name: str) -> SE3:
        '''关节到末端

        :param q: 关节角
        :type q: np.ndarray
        :param fg_name: _description_
        :type fg_name: str
        :return: _description_
        :rtype: SE3
        '''
        qa = np.zeros(20)
        if fg_name in ["fourth", "little"]:
            assert q.shape[0] == 2
            qi3 = self.four_bar_mcp2pip(q[-1], fg_name)
            qi4 = self.four_bar_pip2dip(qi3, fg_name)
            qi = np.append(q, [qi3, qi4])
            # logger.debug(f"{fg_name}: qi3 is {qi3} and qi4 is {qi4}")
        else:
            assert q.shape[0] == 3
            qi4 = self.four_bar_pip2dip(q[-1], fg_name)
            qi = np.append(q, qi4)
            # logger.debug(f"{fg_name}: qi4 is {qi4}")
            
        fg_num = self.get_finger_number(fg_name)
        end_name = f"hand_{self.hand_name_}_f{fg_num}4_end"
        # logger.debug(f"end name is {end_name}")
        qa[4*(fg_num-1):4*fg_num] = qi
        # logger.debug(f"{fg_name}: qa is {qa}")
        
        fk = self.model_rtb_.fkine(qa, end=self.model_rtb_.link_dict[end_name])
        return fk
    
    def fk_q2q_all(self, q: np.ndarray):
        assert q.shape[0] == MOTOR_NUM
        
        qa = np.zeros(20)
        qa[:4] = np.append(q[:3], self.four_bar_pip2dip(q[2], "index"))
        qa[4:8] = np.append(q[3:6], self.four_bar_pip2dip(q[5], "middle"))
        q33 = self.four_bar_mcp2pip(q[7], "fourth")
        q34 = self.four_bar_pip2dip(q33, "fourth")
        qa[8:12] = np.append(q[6:8], [q33, q34])
        q43 = self.four_bar_mcp2pip(q[9], "little")
        q44 = self.four_bar_pip2dip(q43, "little")
        qa[12:16] = np.append(q[8:10], [q43, q44])
        qa[16:] = np.append(q[10:], self.four_bar_pip2dip(q[12], "thumb"))
        
        return qa
    
    def fk_q2e_all(self, q: np.ndarray): #  -> list[SE3]
        assert q.shape[0] == MOTOR_NUM
        
        # TODO: 这样写多求了4次运动学
        return [self.fk_q2e(q[:3], "index"), 
            self.fk_q2e(q[3:6], "middle"), 
            self.fk_q2e(q[6:8], "fourth"), 
            self.fk_q2e(q[8:10], "little"), 
            self.fk_q2e(q[10:], "thumb")
        ]
    
    def ik_e2q(self, pos: np.ndarray, fg_name: str) -> np.ndarray:
        '''使用粒子群算法优化得到单个手指的逆运动学，考虑关节耦合。其中，无名指和小拇指存在多个耦合

        :param pos: _description_
        :type pos: np.ndarray
        :param fg_name: _description_
        :type fg_name: str
        :return: _description_
        :rtype: np.ndarray
        '''
        
        def func(x):
            p = self.fk_q2e(np.asarray(x), fg_name).t
            return np.linalg.norm(p - pos)
        
        if fg_name in ["fourth", "little"]:
            n_dim = 2
        else:
            n_dim = 3
        fg_num = self.get_finger_number(fg_name)
        
        lb = self.qlim_[0,4*(fg_num-1):4*(fg_num-1)+n_dim]
        ub = self.qlim_[1,4*(fg_num-1):4*(fg_num-1)+n_dim]
        logger.debug(f"{fg_name}: fg_num is {fg_num}")
        logger.debug(f"{fg_name}: n_dim is {n_dim}")
        logger.debug(f"{fg_name}: lb is {lb}")
        logger.debug(f"{fg_name}: ub is {ub}")

        # ts = time.time()
        # ga = GA(func=func, n_dim=n_dim, size_pop=20, max_iter=40, prob_mut=0.001, lb=lb, ub=ub, precision=1e-7)
        # best_x, best_y = ga.run()

        # pso = PSO(func=func, dim=n_dim, pop=30, max_iter=80, lb=lb, ub=ub)
        pso = PSO(func=func, dim=n_dim, pop=20, max_iter=40, lb=lb, ub=+ub)
        best_x, best_y = pso.run()
        
        # best_x, best_y = pso(func, lb, ub, swarmsize=20, maxiter=40, minfunc=1e-5)
         
        # logger.info(f"{fg_name}: time is {time.time()-ts}")
        logger.debug('best_x:', best_x, '\n', 'best_y:', best_y)
        return best_x
    
    def ik_e2q_async(self, pos: np.ndarray, fg_name: str) -> np.ndarray:
        return self.ik_e2q(pos, fg_name)
    
    def ik_e2q_all(self, pos_list: list[np.ndarray]) -> np.ndarray:
        '''计算所有手指的关节及逆运动学，末端到关节，按手指编号顺序

        :param pos_list: _description_
        :type pos_list: list[np.ndarray]
        :return: _description_
        :rtype: np.ndarray
        '''
        # data_list = []
        # for key, value in self.finger_mapping.items():
        #     data_list.append((pos_list[value-1], key))
        # pool = multiprocesmath.sing.Pool(processes=5)
        # results = pool.map_async(self.ik_e2q, data_list)
        # results.wait()
        # final_result = results.get()
        # delayed_calls = [delayed(self.ik_e2q)(pos, fg_name) for pos, fg_name in data_list]
        # q = dask.compute(*delayed_calls)
        # q = Parallel(n_jobs=-1)(delayed(self.ik_e2q)(pos, fg_name) for pos, fg_name in data_list)
            
        # async def gather_results():
        #     tasks = []
        #     for key, value in self.finger_mapping.items():
        #         task = asyncio.create_task(self.ik_e2q_async(pos_list[value-1], key))
        #         tasks.append(task)
        #     return await asyncio.gather(*tasks)
        
        # q = asyncio.run(gather_results())

        q = []
        for key, value in self.finger_mapping.items():
            logger.debug(f"Key: {key}, Value: {value}")
            q.append(self.ik_e2q(pos_list[value-1], key))
        q = np.concatenate(q) 
        return q
    
    def ik_e2q_all_free(self, pos_list: list[np.ndarray]):
        # TODO: 手掌具有六自由度，计算合适的位姿与所有关节，避免不同手指的工作空间冲突
        pass
    
    def compute_direction(self, pa, ph):            
            # 计算机械手手指之间的方向向量并归一化
            direction_vectors_robot = []
            for i in range(len(pa)):
                for j in range(i+1, len(pa)):
                    direction_vector = pa[j] - pa[i]
                    normalized_direction_vector = direction_vector / np.linalg.norm(direction_vector)
                    direction_vectors_robot.append(normalized_direction_vector)

            # 计算人手位置之间的方向向量并归一化
            direction_vectors_human = []
            for i in range(len(ph)):
                for j in range(i+1, len(ph)):
                    direction_vector = ph[j] - ph[i]
                    normalized_direction_vector = direction_vector / np.linalg.norm(direction_vector)
                    direction_vectors_human.append(normalized_direction_vector)
            return direction_vectors_robot, direction_vectors_human
        
    def compute_directions_distances(self, p, is_unit=True):
        results = {}

        # Task 1: Compute directions and distances from origin to each point in p
        origin_directions = []
        origin_distances = []
        for point in p:
            direction = point
            distance = np.linalg.norm(direction)
            if is_unit:
                unit_direction = direction / distance  # Unit direction vector
                origin_directions.append(unit_direction)
            else:
                origin_directions.append(direction)
            origin_distances.append(distance)
        
        results['origin_directions'] = np.array(origin_directions)
        results['origin_distances'] = np.array(origin_distances)

        # Task 2: Compute directions and distances from each finger tip to the thumb
        finger_positions = p[:4]  # Finger tip positions
        thumb_position = p[4]  # Thumb position

        finger_to_thumb_directions = []
        finger_to_thumb_distances = []
        for finger_tip in finger_positions:
            direction = thumb_position - finger_tip
            distance = np.linalg.norm(direction)
            if is_unit:
                unit_direction = direction / distance  # Unit direction vector
                finger_to_thumb_directions.append(unit_direction)
            else:
                finger_to_thumb_directions.append(direction)
            finger_to_thumb_distances.append(distance)

        results['finger_to_thumb_directions'] = np.array(finger_to_thumb_directions)
        results['finger_to_thumb_distances'] = np.array(finger_to_thumb_distances)

        # Task 3: Compute directions and distances between finger tips
        finger_to_finger_directions = []
        finger_to_finger_distances = []
        for i in range(4):
            for j in range(i + 1, 4):
                direction = finger_positions[j] - finger_positions[i]
                distance = np.linalg.norm(direction)
                if is_unit:
                    unit_direction = direction / distance  # Unit direction vector
                    finger_to_finger_directions.append(unit_direction)
                else:
                    finger_to_finger_directions.append(direction)
                finger_to_finger_distances.append(distance)
        
        results['finger_to_finger_directions'] = np.array(finger_to_finger_directions)
        results['finger_to_finger_distances'] = np.array(finger_to_finger_distances)

        return results
    
    def vvd(self, vector1,vector2):             
            #vector1,vector2夹角 值域0~180
            degree=math.acos(np.dot(vector1, vector2)/(np.linalg.norm(vector1)*np.linalg.norm(vector2)))
            #degree=degree* (180 / m.pi)
            return degree
    def safe_1(self, degree, initial_degree):          #对食指二关节估计数据的一个安全校验，同时更加准确,弧度制计算
        degree = math.pi-degree + math.radians(initial_degree)
        if degree > math.radians(95):
            degree = math.radians(95)
        if  degree <= 0:
            degree = 0
        # degree = degree**2.5/m.radians(95)**1.5
        return degree
    
    #食指一关节偏差角8度,0~35->0~90
    def safe_2(self, vector1,vector2,matrix1,error_degree,K,initial_degree,max_rangedegree):          #对食指一关节弯曲角估计数据正负号判断与一个安全校验，同时更加准确,弧度制计算
        #向量与平面间的夹角，matrix是3X3矩阵，每一行代表平面上的一个点坐标，三个点确定一个平面 #输入两个向量，平面点矩阵3X3,校准角度,增益，初始角度，最大值范围
        # 计算两个边向量
        V1=matrix1[0]-matrix1[1]
        V2=matrix1[2]-matrix1[0]
        # 计算单位法向量（取两个边向量的叉积）手心朝外方向
        normal_vector = np.cross(V2, V1)/np.linalg.norm(np.cross(V1, V2))   #右手用，左手需要交换叉乘向量位置
        # 计算向量在单位法向量上的投影(有正负)
        projection_vector = np.dot(vector1, normal_vector)         
        degree = math.pi-self.vvd(vector1,vector2)
        degree = degree + math.radians(error_degree)
        if projection_vector < 0:                                #说明是指向手掌反方向的，转角是负值
            degree = (-math.pi+self.vvd(vector1,vector2)-math.radians(error_degree))/(K-0.5)
            degree = K*degree
            degree = degree + math.radians(initial_degree)
        if degree > math.radians(max_rangedegree):
            degree = math.radians(max_rangedegree)
        if degree <= 0:
            degree = 0
    #  degree = degree**4/m.radians(max_rangedegree)**3
        return degree

    def safe_3(self,vector1,vector2,error_degree,K,max_rangedegree):          #对拇指3关节弯曲角估计数据正负号判断与一个安全校验，同时更加准确,弧度制计算
        #向量与平面间的夹角，matrix是3X3矩阵，每一行代表平面上的一个点坐标，三个点确定一个平面 #输入两个向量，平面点矩阵3X3,校准角度,增益，初始角度，最大值范围        
        degree = math.pi-self.vvd(vector1,vector2)
        degree = degree + math.radians(error_degree)
        degree = K*degree
        if degree > math.radians(max_rangedegree):
            degree = math.radians(max_rangedegree)
        if degree <= 0:
            degree = 0
    #  degree = degree**4/m.radians(max_rangedegree)**3
        return degree
        
        
    def ik_e2q_retarget(self, ph, ph_end: list[np.ndarray], q_last=None):
        
        palm_plane1 = np.array([ph[0],
                              ph[5],
                              ph[9]])         #用0 5 9的点坐标代表手掌平面
        palm_plane2 = np.array([ph[0],
                              ph[13],
                              ph[17]])         #用0 13 17的点坐标代表手掌平面
        # vector6_7 = ph[7] - ph[6]
        # vector6_5 = ph[5] - ph[6]
        # vector10_11 = ph[11] - ph[10]
        # vector10_9 = ph[9] - ph[10]
        vector9_10 = ph[10] - ph[9]
        vector9_0 = ph[0] - ph[9]
        vector13_14 = ph[14] - ph[13]
        vector13_0 = ph[0] - ph[13]
        vector17_18 = ph[18] - ph[17]
        vector17_0 = ph[0] - ph[17]
        vector5_6 = ph[6] - ph[5]
        vector5_0 = ph[0] - ph[5]
        # vector5_9 = ph[9] - ph[5]
        vector2_3 = ph[3] - ph[2]
        vector2_1 = ph[1] - ph[2]
        vector1_2 = ph[2] - ph[1]
        vector1_0 = ph[0] - ph[1]
        
        q12 = self.safe_2(vector5_6,vector5_0,palm_plane1,-8,90/35,23.6928,110)
        q22 = self.safe_2(vector9_10,vector9_0,palm_plane1,-3,90/35,23.6928,110)
        q32 = self.safe_2(vector13_14,vector13_0,palm_plane2,-3.5,75/36.5,4.2493,95)
        q42 = self.safe_2(vector17_18,vector17_0,palm_plane2,-6.5,90/84,4.2493,95)
        q52 = self.safe_3(vector1_2,vector1_0,-19,70/13,70),
        q53 = self.safe_3(vector2_3,vector2_1,-4,90/60,90)
        
        # q0 = np.array([0.0, 
        #                  self.safe_2(vector5_6,vector5_0,palm_plane1,-8,90/35,23.6928,110), 
        #                  self.safe_1(self.vvd(vector6_7,vector6_5),-3), 
        #                  0.0, 
        #                  self.safe_2(vector9_10,vector9_0,palm_plane1,-3,90/35,23.6928,110), 
        #                  self.safe_1(self.vvd(vector10_11,vector10_9),-3),
        #                  0.0, 
        #                  self.safe_2(vector13_14,vector13_0,palm_plane2,-3.5,75/36.5,4.2493,95),
        #                  0.0 ,
        #                  self.safe_2(vector17_18,vector17_0,palm_plane2,-6.5,90/84,4.2493,95),
        #                  0.0, 
        #                  self.safe_3(vector1_2,vector1_0,-19,70/13,70),
        #                  self.safe_3(vector2_3,vector2_1,-4,90/60,90)])
        
        # return q0
        
        def func(qp):
            q = np.array([qp[0], q12, qp[1], 
                          qp[2], q22, qp[3],
                          qp[4], q32,
                          qp[5], q42,
                          qp[6], q52, q53])
            fk_all = self.fk_q2e_all(q)
            # pa = [fk.t for fk in fk_all]
            pa_end = [fk.t - np.array([-0.02, 0.00, 0.05]) for fk in fk_all] # 对齐人手mediapipe与机械手原点 0.05
            # direction_vectors_robot, direction_vectors_human = self.get_direction(pa, ph)
            ret_a = self.compute_directions_distances(pa_end, is_unit=False)
            ret_h = self.compute_directions_distances(ph_end, is_unit=True)
            
            scale = 1.
            
            fd = ret_h['origin_distances']
            loss_o = np.sum((ret_a['origin_directions'] - scale * fd[:, np.newaxis] * ret_h['origin_directions'])**2)
            
            fd = ret_h['finger_to_thumb_distances']
            sd = np.ones_like(ret_h['finger_to_thumb_distances'])
            # fd[ret_h['finger_to_thumb_distances'] < 0.02] = 0.001
            # sd[ret_h['finger_to_thumb_distances'] < 0.02] = 200
            loss_t = 0.3 * np.sum(sd[:, np.newaxis] * (ret_a['finger_to_thumb_directions'] - scale * fd[:, np.newaxis] * ret_h['finger_to_thumb_directions'])**2)
            
            fd = ret_h['finger_to_finger_distances']
            sd = np.ones_like(ret_h['finger_to_finger_distances'])
            # fd[ret_h['finger_to_finger_distances'] < 0.03] = 0.03
            # sd[ret_h['finger_to_finger_distances'] < 0.02] = 400
            loss_f = np.sum(sd[:, np.newaxis] * (ret_a['finger_to_finger_directions'] - scale * fd[:, np.newaxis] * ret_h['finger_to_finger_directions'])**2)
            
            loss_q = 2e-2 * np.linalg.norm(q[[1, 3]]) \
                + 7e-3 * np.linalg.norm(q[[0, 2, 4, 5]])

            # loss_q = 2.5e-3 * np.linalg.norm(q[[1,4,7,9,11,12]])
            # loss_q_pip = 0.5e-3 * np.linalg.norm(q[[2,5]])
            # loss_qy = 4e-3 * np.linalg.norm(q[[0, 3, 6, 8]])
            return loss_o+loss_t+loss_f+loss_q
            # return loss_o+loss_f+loss_q
            # # loss1
            # # return np.sum((np.array(direction_vectors_robot) - np.array(direction_vectors_human))**2) + np.linalg.norm(q)
            # # loss2 不行
            # # return np.sum((np.array(direction_vectors_robot) - np.array(direction_vectors_human))**2)
            # # loss3 只加侧摆
            # # qy = np.array([q[0],q[3],[6],[8]])
            # qy = q[[0, 3, 6, 8]]
            # return np.sum((np.array(direction_vectors_robot) - np.array(direction_vectors_human))**2) + np.linalg.norm(qy)
            
            # loss = 0
            # for i in range(len(direction_vectors_human)):
            #     loss += np.sum((np.array(direction_vectors_robot[i]) - np.array(direction_vectors_human[i]))** 2)
        
        # lb = self.qlim_act_[0]
        # ub = self.qlim_act_[1]
        # logger.debug(f"lb is {lb}")
        # logger.debug(f"ub is {ub}")

        # pso = PSO(func=func, dim=MOTOR_NUM, pop=30, max_iter=100, lb=lb, ub=ub)
        # # pso.X = 
        # # pso.cal_y()
        # best_x, best_y = pso.run()
        
        # # best_x, best_y = pso(func, lb, ub, swarmsize=20, maxiter=40, minfunc=1e-5)
         
        # # logger.info(f"{fg_name}: time is {time.time()-ts}")
        # logger.debug('best_x:', best_x, '\n', 'best_y:', best_y)
        # return best_x
        
        lower_bounds = self.qlim_act_[0][[0,2,3,5,6,8,10]]
        upper_bounds = self.qlim_act_[1][[0,2,3,5,6,8,10]]
        # 将上下限转换为 bounds 格式
        bounds = list(zip(lower_bounds, upper_bounds))
        # print(bounds)
        if q_last is not None:
            x0 = q_last[[0,2,3,5,6,8,10]]
        else:
            x0 = np.zeros((MOTOR_NUM))[[0,2,3,5,6,8,10]]
        sol_psu = minimize(func, x0, method='SLSQP', bounds=bounds)
        # SLSQP(快) L-BFGS-B Powell
        if sol_psu.success:
            q_ret = sol_psu.x
            return np.array([q_ret[0], q12, q_ret[1], 
                          q_ret[2], q22, q_ret[3],
                          q_ret[4], q32,
                          q_ret[5], q42,
                          q_ret[6], q52, q53])
            
        else:
            if q_last is not None: 
                return q_last
            else:
                logger.error("Retarget Error!")
                return np.zeros((MOTOR_NUM))
            
    # def ik_e2q_retarget(self, ph, ph_end: list[np.ndarray], q_last=None):
        
    #     def vvd(vector1,vector2):             
    #         #vector1,vector2夹角 值域0~180
    #         degree=math.acos(np.dot(vector1, vector2)/(np.linalg.norm(vector1)*np.linalg.norm(vector2)))
    #         #degree=degree* (180 / m.pi)
    #         return degree
        
    #     def func(q):
    #         fk_all = self.fk_q2e_all(q)
    #         # pa = [fk.t for fk in fk_all]
    #         pa_end = [fk.t - np.array([-0.02, 0.00, 0.05]) for fk in fk_all] # 对齐人手mediapipe与机械手原点 0.05
    #         # direction_vectors_robot, direction_vectors_human = self.get_direction(pa, ph)
    #         ret_a = self.compute_directions_distances(pa_end, is_unit=False)
    #         ret_h = self.compute_directions_distances(ph_end, is_unit=True)
            
    #         scale = 1.
            
    #         fd = ret_h['origin_distances']
    #         loss_o = np.sum((ret_a['origin_directions'] - scale * fd[:, np.newaxis] * ret_h['origin_directions'])**2)
            
    #         fd = ret_h['finger_to_thumb_distances']
    #         sd = np.ones_like(ret_h['finger_to_thumb_distances'])
    #         # fd[ret_h['finger_to_thumb_distances'] < 0.02] = 0.001
    #         # sd[ret_h['finger_to_thumb_distances'] < 0.02] = 200
    #         loss_t = np.sum(sd[:, np.newaxis] * (ret_a['finger_to_thumb_directions'] - scale * fd[:, np.newaxis] * ret_h['finger_to_thumb_directions'])**2)
            
    #         fd = ret_h['finger_to_finger_distances']
    #         sd = np.ones_like(ret_h['finger_to_finger_distances'])
    #         # fd[ret_h['finger_to_finger_distances'] < 0.03] = 0.03
    #         # sd[ret_h['finger_to_finger_distances'] < 0.02] = 400
    #         loss_f = np.sum(sd[:, np.newaxis] * (ret_a['finger_to_finger_directions'] - scale * fd[:, np.newaxis] * ret_h['finger_to_finger_directions'])**2)
            
    #         loss_q = 2.5e-3 * np.linalg.norm(q[[1,2,4,5,7,9,11,12]])
    #         loss_qy = 5e-3 * np.linalg.norm(q[[0, 3, 6, 8]])

    #         # loss_q = 2.5e-3 * np.linalg.norm(q[[1,4,7,9,11,12]])
    #         # loss_q_pip = 0.5e-3 * np.linalg.norm(q[[2,5]])
    #         # loss_qy = 4e-3 * np.linalg.norm(q[[0, 3, 6, 8]])
            
    #         return loss_o+loss_t+loss_f+loss_q+loss_qy
    #         # # loss1
    #         # # return np.sum((np.array(direction_vectors_robot) - np.array(direction_vectors_human))**2) + np.linalg.norm(q)
    #         # # loss2 不行
    #         # # return np.sum((np.array(direction_vectors_robot) - np.array(direction_vectors_human))**2)
    #         # # loss3 只加侧摆
    #         # # qy = np.array([q[0],q[3],[6],[8]])
    #         # qy = q[[0, 3, 6, 8]]
    #         # return np.sum((np.array(direction_vectors_robot) - np.array(direction_vectors_human))**2) + np.linalg.norm(qy)
            
    #         # loss = 0
    #         # for i in range(len(direction_vectors_human)):
    #         #     loss += np.sum((np.array(direction_vectors_robot[i]) - np.array(direction_vectors_human[i]))** 2)
        
    #     # lb = self.qlim_act_[0]
    #     # ub = self.qlim_act_[1]
    #     # logger.debug(f"lb is {lb}")
    #     # logger.debug(f"ub is {ub}")

    #     # pso = PSO(func=func, dim=MOTOR_NUM, pop=30, max_iter=100, lb=lb, ub=ub)
    #     # # pso.X = 
    #     # # pso.cal_y()
    #     # best_x, best_y = pso.run()
        
    #     # # best_x, best_y = pso(func, lb, ub, swarmsize=20, maxiter=40, minfunc=1e-5)
         
    #     # # logger.info(f"{fg_name}: time is {time.time()-ts}")
    #     # logger.debug('best_x:', best_x, '\n', 'best_y:', best_y)
    #     # return best_x
        
    #     lower_bounds = self.qlim_act_[0]
    #     upper_bounds = self.qlim_act_[1]
    #     # 将上下限转换为 bounds 格式
    #     bounds = list(zip(lower_bounds, upper_bounds))
    #     # print(bounds)
    #     if q_last is not None:
    #         x0 = q_last
    #     else:
    #         x0 = np.zeros((MOTOR_NUM))
    #     sol_psu = minimize(func, x0, method='SLSQP', bounds=bounds)
    #     # SLSQP(快) L-BFGS-B Powell
    #     if sol_psu.success:
    #         return sol_psu.x
    #     else:
    #         if q_last is not None: 
    #             return q_last
    #         else:
    #             logger.error("Retarget Error!")
    #             return np.zeros((MOTOR_NUM))
     
    # %% 用于计算驱动空间与关节空间的运动学映射  
    def ik_q2d_m1(self, q: np.ndarray, fg_name: str) -> np.ndarray:
        '''用于求解食指,中指的逆运动学,其中第三个关节总是指定为直驱关节角度

        :param q: _description_
        :type q: np.ndarray
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        assert q.shape[0] == 3
        assert (fg_name=="index" or fg_name=="middle")
        cfg = self.get_module_config(fg_name)
        
        # psi = self.hand_dir_*q[0]
        psi = q[0]
        theta = q[1]
        phi = q[2]
        # logger.debug(f"psi: {psi}")
        # logger.debug(f"theta: {theta}")
        # logger.debug(f"phi: {phi}")
        
        # MCP
        rx = np.array([[1, 0, 0],
                        [0,math.cos(psi),-math.sin(psi)],
                        [0,math.sin(psi),math.cos(psi)]])    
        ry = np.array([[math.cos(theta),0,math.sin(theta)],
                        [0,1,0],
                        [-math.sin(theta),0,math.cos(theta)]])
        # r = rx @ ry
        r = rx @ ry
        p1r = r @ cfg["p1"]
        p2r = r @ cfg["p2"]
        # logger.debug(f"p1r: {p1r}")
        
        d_left  = -(p1r[2] - math.sqrt(cfg["l1"]**2 - np.sum((p1r[:2]-cfg["p3"][:2])**2)))
        d_right = -(p2r[2] - math.sqrt(cfg["l2"]**2 - np.sum((p2r[:2]-cfg["p4"][:2])**2)))
        logger.debug(f"d_left: {d_left}")
        logger.debug(f"d_right: {d_right}")
        d_left  -= cfg["z0"]
        d_right -= cfg["z0"]
        # print(d_left)
        # d = np.array([d_left, d_right, phi*cfg["bevel"]])
        # if self.is_left_:
        #     d = np.array([d_left/cfg["h"], d_right/cfg["h"], phi/(2*math.pi)])
        # else:
        d = np.array([d_right/cfg["h"], d_left/cfg["h"], phi/(2*math.pi)])
        return d 
        
    def ik_q2d_m2(self, q: np.ndarray, fg_name: str) -> np.ndarray:
        '''用于求解无名指与小拇指的逆运动学,其中第一个关节总是指定为直驱关节角度

        :param q: _description_
        :type q: np.ndarray 
        :param name: _description_
        :type name: str
        '''
        assert q.shape[0] == 2
        assert (fg_name=="fourth" or fg_name=="little")
        cfg = self.get_module_config(fg_name)
        psi = q[0]
        theta = q[1]
        bevel = psi / (2*math.pi)
        
        rx = np.array([[1, 0, 0],
                        [0,math.cos(psi),-math.sin(psi)],
                        [0,math.sin(psi),math.cos(psi)]])    
        ry = np.array([[math.cos(theta),0,math.sin(theta)],
                        [0,1,0],
                        [-math.sin(theta),0,math.cos(theta)]])
        r = rx @ ry  # 需要交换吗,决定了哪个高哪个低
        p1r = r @ cfg["p1"]
        p2r = r @ cfg["p2"]
        
        def ik_psu(x): # 求根公式太复杂
            p3 = cfg["p3"]
            p3[2] = -x[0] # -Z
            p4 = np.array([x[1], x[2], x[3]])
            eq = p3-p4-x[4]*p2r-x[5]*(p1r - p4)
            
            return [np.linalg.norm(p1r - p4) - cfg["l1"],
                    np.linalg.norm(p3 - p4) - cfg["l2"],
                    np.dot(p1r - p4,p2r),
                    eq[0],
                    eq[1],
                    eq[2]]
        
        sol_pss = fsolve(ik_psu, np.array([0.04, 0.02, 0.0, -0.02, 0.0, -3.10]))
        logger.debug(f"sol_pss: {sol_pss}")
        d = sol_pss[0]-0.01
        
        # ts = time.time()
        # d1 = trans2_theta_psi_D(theta,psi)/1000
        # if np.linalg.norm(d-d1) > 0.001:
        #     print("error")
        # print(d-d1)
        # logger.debug(f"d2_time: {time.time() -ts}")
        # logger.debug(f"d: {d}")
        # print(d)
        # logger.debug(f"d2: {d2}")
        # logger.debug(f"d_direct: {bevel}")
        
        d = np.array([bevel,d/cfg["h"]])
        return d
    
    def ik_q2d_m3(self, q: np.ndarray, fg_name: str) -> np.ndarray:
        '''用于求解大拇指的逆运动学,一关节为侧摆，二关节为弯曲，第三个关节为直驱关节角度

        :param q: _description_
        :type q: np.ndarray
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        assert q.shape[0] == 3
        assert (fg_name=="thumb")
        cfg = self.get_module_config(fg_name)
        psi = q[0]
        theta = q[1]
        phi = q[2]
        
        bevel = psi / (2*math.pi)
        # d1是抬升
        d1=cfg["B1"]-cfg["A1"]*math.tan(cfg["degree1_init1"]-theta)
        # d2是弯曲
        d2=cfg["B2"]-cfg["A2"]*math.tan(cfg["degree1_init2"]-phi)
        d = np.array([bevel, d1/cfg["h"], d2/cfg["h"]])
        return d
    
    def ik_q2d(self, q: np.ndarray, fg_name: str) -> np.ndarray:
        if fg_name in ["index", "middle"]:
            return self.ik_q2d_m1(q, fg_name)
        elif fg_name in ["fourth", "little"]:
            return self.ik_q2d_m2(q, fg_name)
        elif fg_name in ["thumb"]:
            return self.ik_q2d_m3(q, fg_name)
        else:
            logger.error("手指名称错误")
            exit()
    
    def ik_q2d_all(self, q: np.ndarray) -> np.ndarray:
        assert q.shape[0] == 13    
        # 这个地方统一写电机圈数，然后在map里乘对应编码器减速比等    
        d = [self.ik_q2d(q[:3],"index"), 
                      self.ik_q2d(q[3:6],"middle"), 
                      self.ik_q2d(q[6:8],"fourth"), 
                      self.ik_q2d(q[8:10],"little"), 
                      self.ik_q2d(q[10:],"thumb")]
        logger.debug(d)
        d = np.concatenate(d)
        return d
                
    def fk_d2q_m1(self, d: np.ndarray, fg_name: str) -> np.ndarray:
        '''用于求解食指,中指以及大拇指的正运动学,其中d的第三维度总是指定为直驱关节,尽管大拇指的直驱位置与食指不同

        :param d: _description_
        :type d: np.ndarray
        :param name: _description_
        :type name: str
        :return: _description_
        :rtype: _type_
        '''
        assert d.shape[0] == 3
        assert (fg_name=="index" or fg_name=="middle")
        cfg = self.get_module_config(fg_name)
        d_right = d[0] # 定义求出的电机位移始终为+,便于发送电机指令,但坐标系中为负数
        d_left = d[1]
        d_direct = d[2]
        
        # pip
        pip = d_direct * (2*math.pi) 
        
        # dip,四杆机构
        dip = self.four_bar_pip2dip(pip, fg_name)

        # mcp
        d_left  *= cfg["h"]
        d_right *= cfg["h"]
        d_left  += cfg["z0"]
        d_right += cfg["z0"]
        d_left = -d_left # TODO: 有负号
        d_right = -d_right
        logger.debug(f"d_left: {d_left}")
        logger.debug(f"d_right: {d_right}")
        
        def fk_pss(x):
            rx = np.array([[1, 0, 0],
                            [0,cos(x[0]),-sin(x[0])],
                            [0,sin(x[0]),cos(x[0])]])
            ry = np.array([[cos(x[1]),0,sin(x[1])],
                            [0,1,0],
                            [-sin(x[1]),0,cos(x[1])]])
            r = rx @ ry
            # r = ry @ rx 
            p1r = r @ cfg["p1"]
            p2r = r @ cfg["p2"]
            p3 = cfg["p3"]
            p3[2] = d_left
            p4 = cfg["p4"]
            p4[2] = d_right
            return [np.linalg.norm(p1r - p3) - cfg["l1"],
                    np.linalg.norm(p2r - p4) - cfg["l2"]]
                       
        sol_pss = fsolve(fk_pss, np.array([math.radians(0), math.radians(50)]))
        logger.debug(f"sol_pss: {sol_pss}")
        psi = sol_pss[0]
        theta = sol_pss[1]
        q = np.array([psi, theta, pip, dip])
        return q
    
    def fk_d2q_m2(self, d: np.ndarray, fg_name: str) -> np.ndarray:
        '''用于求解无名指与小拇指的正运动学,其中不包含直驱关节。主要用于显示/规划电机空间使得满足末端轨迹

        :param q: _description_
        :type q: np.ndarray
        :param name: _description_
        :type name: str
        '''
        assert d.shape[0] == 2
        assert (fg_name=="fourth" or fg_name=="little")
        cfg = self.get_module_config(fg_name)
        
        q1 = d[0] * (2*math.pi)
        d = d[1] 
        d *= cfg["h"]
        d += 0.01
        
        rx = np.array([[1, 0, 0],
                        [0,math.cos(q1),-math.sin(q1)],
                        [0,math.sin(q1),math.cos(q1)]])    
        def fk_psu(x):
            ry = np.array([[cos(x[0]),0,sin(x[0])],
                            [0,1,0],
                            [-sin(x[0]),0,cos(x[0])]])
            r = rx @ ry
            p1r = r @ cfg["p1"]
            p2r = r @ cfg["p2"]
            
            p3 = cfg["p3"]
            p3[2] = -d # -Z
            p4 = np.array([x[1], x[2], x[3]])
            eq = p3-p4-x[4]*p2r-x[5]*(p1r - p4)
            
            return [np.linalg.norm(p1r - p4) - cfg["l1"],
                    np.linalg.norm(p3 - p4) - cfg["l2"],
                    np.dot(p1r - p4,p2r),
                    eq[0],
                    eq[1],
                    eq[2]]
        
        x0 = np.array([math.radians(50), 0.02, 0.0, 0.001, 0.0, -3.10])
        sol_psu = fsolve(fk_psu, x0)
        
        logger.debug(f"sol_psu: {sol_psu}")
        # print(sol_psu[0])
        # q2 = math.atan2(sol_psu.x[1],sol_psu.x[0])
        # print(q2)
        
        q2 = sol_psu[0]
    
        pip = self.four_bar_mcp2pip(q2, fg_name)    
        dip = self.four_bar_pip2dip(pip, fg_name)

        q = np.array([q1, q2, pip, dip])
        return q
        
    def fk_d2q_m3(self, d: np.ndarray, fg_name: str) -> np.ndarray:
        pass      
    
    def fk_d2q(self, d: np.ndarray, fg_name: str) -> np.ndarray:
        if fg_name in ["index", "middle"]:
            return self.fk_d2q_m1(d, fg_name)
        elif fg_name in ["fourth", "little"]:
            return self.fk_d2q_m2(d, fg_name)
        elif fg_name in ["thumb"]:
            return self.fk_d2q_m3(d, fg_name)
        else:
            logger.error("手指名称错误")
            exit()      
    
    def fk_d2q_all(self, d: np.ndarray) -> np.ndarray:
        # fk,将给出所有耦合关节的参数
        pass

    # %% 轨迹规划    
    def plan_in_dspace(self, t0, q0, v0, ac0, tf, qf, vf, acf, n):
        M = [[1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]]
        M = np.matrix(M)
        b = [[q0], [v0], [ac0], [qf], [vf], [acf]]
        a = np.linalg.inv(M) * b
        # x = np.arange(t0,tf,0.05)
        x = np.linspace(t0, tf, n)

        def qt(t,c0,c1,c2,c3,c4,c5):
            return c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5
        
        def vt(t,c1,c2,c3,c4,c5):
            return c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4
        
        def at(t,c2,c3,c4,c5):
            return 2*c2 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3

        q = qt(x,a[0,0],a[1,0],a[2,0],a[3,0],a[4,0],a[5,0])
        v = vt(x,a[1,0],a[2,0],a[3,0],a[4,0],a[5,0])
        # ac = at(x,a[2,0],a[3,0],a[4,0],a[5,0])
        # return q,v,ac
        return q,v
    
    def plan_in_dspace_all(self, t0, q0, v0, ac0, tf, qf, vf, acf, n):
        # assert q0.shape[0] == 13
        q = []
        v = []
        for i in range(q0.shape[0]):
            qi, vi = self.plan_in_dspace(t0, q0[i], v0[i], ac0[i], tf, qf[i], vf[i], acf[i], n)
            q.append(qi)
            v.append(vi)
        
        q=np.vstack(q)
        v=np.vstack(v)
        return q, v
    
    def plan_in_dspace_w_ptraj(self, ptraj):
        # TODO: 驱动空间规划,使其趋近于给定的末端空间
        pass
            
# %% main  
if __name__ == '__main__':
    hand_file = '/home/ccs/ros2/libertech_ws/src/libertech_ros2_robots/libertech_hand_description/urdf/hand_right.xacro.urdf'
    
    model = HandModelKinematics(hand_file)

    print(model.fk_q2e(np.array([0,1,1]), "index"))
    
    # print(model.fk_q2e_all(np.zeros(13)))
    
    pos = np.array([0.03841,0.02866,0.08211])
    # p = model.fk_q2e(np.array([0,0,1]), "index").o
    # print(np.linalg.norm(p - pos))
    
    # q = model.ik_e2q(pos, "index")
    
    # print(model.fk_q2e(np.asarray(q), "index"))
    # print(model.fk_q2e(np.asarray(q), "index").t)
    # print(model.fk_q2e(np.array([0.309643, 2.055571, 1.472229]), "index").a)
    
    print('='*30)
    
    se3_all = model.fk_q2e_all(np.array([0,1,1,
                                     0,1,1,
                                     0,1,
                                     0,1,
                                     0,1,1]))
    print(se3_all)
    
    p_all = [se3.t for se3 in se3_all]
    # p_all = np.concatenate(p_all) 
    # print(p_all)
    
    ts = time.time()
    print(model.ik_e2q_all(p_all))
    logger.info(f"time is {time.time()-ts}")
    
    # q = np.array([0.0, 0.3, 0.0, 0.0, 1.7, 1.5, 0.0, 1.3, 0.0, 1.3, 1.2, 0.1, 0.9])
    # logger.info(f"q is {q}")
    # d= model.ik_q2d_all(q)
    # logger.info(f"d is {d}")
    
    # d_traj, v_traj = model.plan_in_dspace_all(0,np.zeros_like(d),np.zeros_like(d),np.zeros_like(d),
    #                                   1,d,np.zeros_like(d),np.zeros_like(d),30)
    # logger.info(f"d_traj is {d_traj}")
    # logger.info(f"d_traj.shape[1] is {d_traj.shape[1]}")
    # logger.info(f"pos: {d_traj[:,-1]}") 
    
    
    # print(model.ik_q2d(np.array([0,0,0]), 'thumb'))
    
    # %% 测试时间
    # N = 100
    # ts = time.time()
    # for _ in range(N):
    #     model.fk_q2e(np.array([0,1,1]), "index")
    # logger.info(f"fk_q2e(index) 用时为: {1000*(time.time()-ts)/N} ms")
    
    # ts = time.time()
    # for _ in range(N):
    #     model.fk_q2e(np.array([0,1,1]), "thumb")
    # logger.info(f"fk_q2e(thumb) 用时为: {1000*(time.time()-ts)/N} ms")
    
    # ts = time.time()
    # for _ in range(N):
    #     se3_all = model.fk_q2e_all(np.array([0,1,1,
    #                                  0,1,1,
    #                                  0,1,
    #                                  0,1,
    #                                  0,1,1]))
    # logger.info(f"fk_q2e_all() 用时为: {1000*(time.time()-ts)/N} ms")

    # p_all = []
    # for se3 in se3_all:
    #     p_all.append(se3.t)
    
    # ts = time.time()
    # for _ in range(N):
    #     model.ik_e2q_all(p_all)
    # logger.info(f"ik_e2q_all() 用时为: {1000*(time.time()-ts)/N} ms")
    
    # ts = time.time()
    # for _ in range(N):
    #     model.ik_q2d(np.array([0,1,1]), "index")
    # logger.info(f"ik_q2d(index) 用时为: {1000*(time.time()-ts)/N} ms")
    
    # ts = time.time()
    # for _ in range(N):
    #     model.ik_q2d(np.array([0,1]), "fourth")
    # logger.info(f"ik_q2d(fourth) 用时为: {1000*(time.time()-ts)/N} ms")
    
    # ts = time.time()
    # for _ in range(N):
    #     model.ik_q2d(np.array([0,1,1]), "thumb")
    # logger.info(f"ik_q2d(thumb) 用时为: {1000*(time.time()-ts)/N} ms")
    
    # ts = time.time()
    # for _ in range(N):
    #     model.ik_q2d_all(np.array([0,1,1,
    #                                  0,1,1,
    #                                  0,1,
    #                                  0,1,
    #                                  0,1,1]))
    # logger.info(f"ik_q2d_all() 用时为: {1000*(time.time()-ts)/N} ms")
    
    q2_sequence = np.arange(0, 1.75, 0.01)
    # print(q2_sequence)
    
    # pos = np.array([0.4, 0.0, 0.0])
    # n_error = 0
    # for q2 in q2_sequence:
    #     pos[1] = q2
    #     logger.info(f"pos为: {pos}")
        
    #     ts = time.time()
    #     d = model.ik_q2d(pos, "index")
    #     logger.info(f"用时为: {1000*(time.time()-ts)} ms, ik_q2d()为: {d}")
        
    #     ts = time.time()
    #     pos_fk = model.fk_d2q(d, "index")
    #     logger.info(f"用时为: {1000*(time.time()-ts)} ms, fk_d2q()为: {pos_fk}")
        
    #     logger.info(f"pos误差为: {pos - pos_fk[:3]}")
    #     if np.linalg.norm(pos - pos_fk[:3]) > 0.05:
    #         n_error += 1
            
    # print(n_error)
        
    # n_error = 0
    # pos = np.array([-0.4, 0.0])
    # for q2 in q2_sequence:
    #     pos[1] = q2
    #     logger.info(f"pos为: {pos}")
        
    #     ts = time.time()
    #     d = model.ik_q2d(pos, "fourth")
    #     logger.info(f"用时为: {1000*(time.time()-ts)} ms, ik_q2d()为: {d}")
        
    #     ts = time.time()
    #     pos_fk = model.fk_d2q(d, "fourth")
    #     logger.info(f"用时为: {1000*(time.time()-ts)} ms, fk_d2q()为: {pos_fk}")
    #     print(np.linalg.norm(pos - pos_fk[:2]))
    #     if np.linalg.norm(pos - pos_fk[:2]) > 0.05:
    #         n_error += 1
            
    # print(n_error)
        
    # # print(model.n_e)
    
    
    q1_sequence = np.arange(-0.4, 0.4, 0.05)
    q2_sequence = np.arange(0, 1.75, 0.05)
    
    print(f'---------------测试食指、中指---------------')
    n_error = 0
    pos = np.array([0.0, 0.0, 0.4])
    for q1 in q1_sequence:
        for q2 in q2_sequence:
            pos[0] = q1
            pos[1] = q2
            # print(f"设定的关节角为: {pos}")
            
            ts = time.time()
            d = model.ik_q2d(pos, "index")
            # print(f"用时为: {1000*(time.time()-ts)} ms, ik_driver3()为: {d}")
            
            ts = time.time()
            pos_fk = model.fk_d2q(d, "index")
            # print(f"用时为: {1000*(time.time()-ts)} ms, fk_driver3()为: {pos_fk}")
            
            # print(f"pos误差为: {pos - pos_fk[:3]}")
            if np.linalg.norm(pos - pos_fk[:3]) > 0.05:
                n_error += 1
          
    print(f"ik求解错误次数为: {n_error}")  
    
    print(f'---------------测试无名指、小拇指---------------')
    n_error = 0
    
    q1_sequence = np.arange(-0.4, 0.4, 0.05)
    q2_sequence = np.arange(0, 1.75, 0.05)
    
    pos = np.array([0.0, 0.0])
    for q1 in q1_sequence:
        for q2 in q2_sequence:
            pos[0] = q1
            pos[1] = q2
            # print(f"设定的关节角为: {pos}")
            
            ts = time.time()
            d = model.ik_q2d(pos, "fourth")
            # print(f"用时为: {1000*(time.time()-ts)} ms, ik_driver2()为: {d}")
            
            ts = time.time()
            pos_fk = model.fk_d2q(d, "fourth")
            # print(f"用时为: {1000*(time.time()-ts)} ms, fk_driver2()为: {pos_fk}")
            
            # print(f"pos误差为: {pos - pos_fk[:2]}")
            if np.linalg.norm(pos - pos_fk[:2]) > 0.05:
                print(f"设定的关节角为: {pos}")
                print(f"用时为: {1000*(time.time()-ts)} ms, ik_driver2()为: {d}")
                print(f"用时为: {1000*(time.time()-ts)} ms, fk_driver2()为: {pos_fk}")
                print(f"pos误差为: {pos - pos_fk[:2]}")
                n_error += 1
                
    print(f"ik求解错误次数为: {n_error}")