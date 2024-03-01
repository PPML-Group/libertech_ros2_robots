from roboticstoolbox.robot.ERobot import ERobot
import numpy as np
from spatialmath import SO3, SE3, UnitQuaternion
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import math
import time
from sko.PSO import PSO
# from sko.GA import GA
from pyswarm import pso
from hand_cfg import *
from hand_model_cal import trans2_theta_psi_D    
import asyncio
import threading
from typing import Tuple
from logger import logger
import urdf_parser_py.urdf as urdf
from math import cos, sin
from scipy.optimize import root, fsolve

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
        self.qname_ = self.model_rtb_.joint_names_[2:]
        self.qname_act_ = []
        for i in range(5):
            self.qlim_act_.append(self.qlim_[:, 4*i:4*i+list(self.finger_driver_num.items())[i][1]])
            self.qname_act_.extend(self.qname_[4*i:4*i+list(self.finger_driver_num.items())[i][1]])
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
        return degree2_init-(math.acos(parameter2)-math.acos(parameter1))
        
    def four_bar_backward(self, q, A, B, C, D, degree1_init, degree2_init):
        L2=math.sqrt(C**2+D**2-2*C*D*math.cos(degree2_init-q))
        parameter1=math.sin(degree2_init-q)*D/L2
        parameter2=(L2^2+A^2-B^2)/(2*L2*A)
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
        end_name = f"{self.hand_name_}_f{fg_num}4"
        # logger.debug(f"end name is {end_name}")
        qa[4*(fg_num-1):4*fg_num] = qi
        # logger.debug(f"{fg_name}: qa is {qa}")
        
        fk = self.model_rtb_.fkine(qa, end=self.model_rtb_.link_dict[end_name])
        return fk
    
    def fk_q2e_all(self, q: np.ndarray): #  -> list[SE3]
        assert q.shape[0] == MOTOR_NUM
        
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

        pso = PSO(func=func, n_dim=n_dim, pop=20, max_iter=40, lb=lb, ub=ub)
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
        
        # ts = time.time()
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
        
        sol_pss = fsolve(ik_psu, np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]))
        logger.debug(f"sol_pss: {sol_pss}")
        d = sol_pss[0]-0.01
        print(d)
        
        # def ik_psu(x): # 求根公式太复杂
        #     l2 = np.array([x[0], x[1], x[2]])
        #     p3 = cfg["p3"]
        #     p3[2] = x[3]
        #     l1 = p1r - p3 - l2
        #     equ = l2 - np.dot(l2,p2r)*p2r - np.dot(l1,l2)*l1/cfg["l1"]
            
        #     return [equ[0],
        #             equ[1],
        #             equ[2],
        #             np.linalg.norm(l2)-cfg["l2"]]
    
        # sol_psu = fsolve(ik_psu, np.array([0, 0, 0, 0]))
        # logger.debug(f"sol_psu: {sol_psu}")
        # d = sol_psu[3]-0.1
        # print(d)
        # logger.debug(f"d_time: {time.time() -ts}")
        
        # ts = time.time()
        d = trans2_theta_psi_D(theta,psi)/1000
        # print(d)
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
        q2 = 0
        
        # rx = np.array([[1, 0, 0],
        #                 [0,math.cos(q1),-math.sin(q1)],
        #                 [0,math.sin(q1),math.cos(q1)]])    
        # def fk_psu(x):
        #     ry = np.array([[cos(x[0]),0,sin(x[0])],
        #                     [0,1,0],
        #                     [-sin(x[0]),0,cos(x[0])]])
        #     r = rx @ ry
        #     # r = ry @ rx 
        #     p1r = r @ cfg["p1"]
        #     p2r = r @ cfg["p2"]
            
        #     p3 = cfg["p3"]
        #     p3[2] = -d[1] # -Z
            
        #     p4 = np.array([x[1], x[2], x[3]])
        #     eq = p3-p4-x[4]*p2r-x[5]*(p1r - p4)
            
        #     return [np.linalg.norm(p1r - p4) - cfg["l1"],
        #             np.linalg.norm(p3 - p4) - cfg["l2"],
        #             np.dot(p1r - p4,p2r),
        #             eq[0],
        #             eq[1],
        #             eq[2]]
            
        # sol_pss = fsolve(fk_psu, np.array([math.radians(0), 0,0,0,0,0]))
        # logger.debug(f"sol_pss: {sol_pss}")
        # q2 = sol_pss[0]
    
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

    # print(model.fk_q2e(np.array([0,1,1]), "index"))
    
    # # print(model.fk_q2e_all(np.zeros(13)))
    
    # pos = np.array([0.03841,0.02866,0.08211])
    # # p = model.fk_q2e(np.array([0,0,1]), "index").o
    # # print(np.linalg.norm(p - pos))
    
    # # q = model.ik_e2q(pos, "index")
    
    # # print(model.fk_q2e(np.asarray(q), "index"))
    # # print(model.fk_q2e(np.asarray(q), "index").t)
    # # print(model.fk_q2e(np.array([0.309643, 2.055571, 1.472229]), "index").a)
    
    # print('='*30)
    
    # se3_all = model.fk_q2e_all(np.array([0,1,1,
    #                                  0,1,1,
    #                                  0,1,
    #                                  0,1,
    #                                  0,1,1]))
    # print(se3_all)
    
    # p_all = []
    # for se3 in se3_all:
    #     p_all.append(se3.t)
    # # p_all = np.concatenate(p_all) 
    # # print(p_all)
    
    # ts = time.time()
    # print(model.ik_e2q_all(p_all))
    # logger.info(f"time is {time.time()-ts}")
    
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
    
    q2_sequence = np.arange(0, 1.95, 0.2)
    print(q2_sequence)
    
    pos = np.array([0.4, 0.0, 0.0])
    n_error = 0
    for q2 in q2_sequence:
        pos[1] = q2
        logger.info(f"pos为: {pos}")
        
        ts = time.time()
        d = model.ik_q2d(pos, "index")
        logger.info(f"用时为: {1000*(time.time()-ts)} ms, ik_q2d()为: {d}")
        
        ts = time.time()
        pos_fk = model.fk_d2q(d, "index")
        logger.info(f"用时为: {1000*(time.time()-ts)} ms, fk_d2q()为: {pos_fk}")
        
        logger.info(f"pos误差为: {pos - pos_fk[:3]}")
        if np.linalg.norm(pos - pos_fk[:3]) > 0.05:
            n_error += 1
            
    print(n_error)
        
    # pos = np.array([0.0, 0.0])
    # for q2 in q2_sequence:
    #     pos[1] = q2
    #     logger.info(f"pos为: {pos}")
        
    #     ts = time.time()
    #     d = model.ik_q2d(pos, "fourth")
    #     logger.info(f"用时为: {1000*(time.time()-ts)} ms, ik_q2d()为: {d}")
        
    #     ts = time.time()
    #     pos_fk = model.fk_d2q(d, "fourth")
    #     logger.info(f"用时为: {1000*(time.time()-ts)} ms, fk_d2q()为: {pos_fk}")
        
    