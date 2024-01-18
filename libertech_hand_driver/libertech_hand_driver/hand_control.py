import asyncio
import numpy as np
from queue import Queue
from typing import Optional
import traceback
import yaml
from libertech_hand_driver.hand_model_kinematics import HandModelKinematics
from libertech_hand_driver.CommProtocol import *
from libertech_hand_driver.hand_cfg import *
from libertech_hand_driver.logger import logger
import libertech_hand_driver.TinyFrame as TF
import time

np.set_printoptions(precision=3, suppress=True)

class HandAPI:
    def __init__(self, 
                 hand_path,
                 actions_path='actions.yaml',
                 mode='p_p', # p_p
                 target_ip='10.10.20.61',
                 target_port=2000,
                 display=True
                 ):
        # 思路：
        # 1. 末端/关节只有位置指令，不存在直接给出电机空间或关节空间的轨迹指令。底层可以力指令或位置指令，但只提供位置接口
        # 2. self.cmd_在外部被更新，注意线程锁；self.data_用于力控模式反馈和显示
        # 3. 单点位置控制总是会给出一条电机空间的轨迹
        # 4. 若需要末端轨迹规划，则需要优化一条电机空间的轨迹，使得接近给出的末端轨迹
        # 5. send_cmds中最好写为定时函数
        # 6. 存在两种模式：位置模式与力控模式。
        #   位置模式：给出的轨迹直接用于更新send_cmds的qDes，qdDes；同时可更改kpDes，kdDes，此时存在稳态误差
        #       也可以通过增加
        #   力控模式：给出的轨迹经过力转换模块，使用无模型的PID/PD算法给出，使用PID时
        # 7. 手指与外界交互时，使用导纳控制（存在触觉传感），根据接触程度修正末端偏差，然后进一步修正电机位置指令
        #   此时，使用力控模式的PD会比较好
        
        self.hand_model_ = HandModelKinematics(hand_path, display)
        self.hand_dir_ = self.hand_model_.hand_dir_
        self.hand_name_ = self.hand_model_.hand_name_  
        # logger.info(f"{self.hand_name_}")
        with open(actions_path, 'r') as file:
            self.actions = yaml.safe_load(file)
        self.motor_mapping_ = motor_mapping[self.hand_name_]
        self.motor_direction_ = motor_direction[self.hand_name_]
        logger.info(self.motor_direction_)

        self.mode_ = mode
        self.plan_ = 1
        self.target_ip_ = target_ip
        self.target_port_ = target_port
        self.reader = None
        self.writer = None      
        self.enable_rx_ = True
        self.rx_cnt = 0  
        self.data_ = FingerData()   
        
        self.cmd_pt_ = FingerCommand()
        self.error_sum = np.zeros(MOTOR_NUM)
        
        if self.mode_ == 'p_p':
            logger.info("使用位置模式")
            self.duration_send_ = 0.005
            if self.plan_ == 1:
                # self.motor_kp_ = 2*np.array([3, 5, 5, 2, 2, 2, 4, 4, 2, 2, 2, 6, 6])
                self.motor_kp_ = 2*np.array([3, 5, 5, 2, 2, 1, 3, 3, 1, 1, 1, 6, 6])
                self.motor_kd_  = 3*np.array([2, 2, 2, 2, 2, 8, 2, 2, 8, 8, 8, 2, 2]) 
                # self.motor_kp_  = np.ones(MOTOR_NUM) * 0.5
                # self.motor_kd_  = np.ones(MOTOR_NUM) * 0.5 
            else:
                self.motor_kp_  = np.array([3, 5, 5, 2, 2, 2, 3, 3, 2, 2, 2, 6, 6])
                # self.motor_kp_  = np.array([3, 5, 5, 2, 2, 0.5, 3, 3, 0.5, 0.5, 0.5, 6, 6])
                self.motor_kd_  = np.array([2, 2, 2, 2, 2, 8, 2, 2, 8, 8, 8, 2, 2])
                # self.motor_kp_  = np.ones(MOTOR_NUM) * 0.5
                # self.motor_kd_  = np.ones(MOTOR_NUM) * 0.5 
            
            self.motor_ki_  = 0 * np.ones(MOTOR_NUM)
        elif self.mode_ == 'p_t':
            # logger.info("使用力控模式")
            logger.error("力控模式暂不使用")
            self.duration_send_ = 0.005/MOTOR_NUM
            # self.motor_kp_  = np.array([3, 5, 5, 2, 2, 1, 3, 3, 1, 1, 1, 3, 3]) * 1
            self.motor_kp_  = 2 * np.ones(MOTOR_NUM)
            self.motor_kd_  = 0.00 * np.ones(MOTOR_NUM)
            self.motor_ki_  = 0.002 * np.ones(MOTOR_NUM)
            
        # 对每个手指创建一个队列，在外部设置这个队列，暂时分开，写一起可能会冲突
        self.queue_index_ = Queue()
        self.queue_middle_ = Queue()
        self.queue_fourth_ = Queue()
        self.queue_little_ = Queue()
        self.queue_thumb_ = Queue()
                   
    # ############################ 用户接口 ############################     
    async def init(self):
        logger.info("初始化...")
        await self.connect()
        logger.info("已连接")
        
        self.tf = TF.TinyFrame()
        self.tf.SOF_BYTE = 0x01
        self.tf.ID_BYTES = 0x01
        self.tf.LEN_BYTES = 0x02
        self.tf.TYPE_BYTES = 0x01
        self.tf.CKSUM_TYPE = 'crc16'
        
        self.tf.write = self.tf
        self.tf.write_async = self.tf_write_async
        self.tf.add_fallback_listener(self.tf_general_listener)
            
        # 电机初始参数   
        # self.motor_dir_ = [-1, 1, 1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1]
        # self.enc_dir_   = [1, 0, 0, 0, 1, 0, 0, 0, 1, 1,  1,  0,  0]
        await self.reset_id()
        for i in range(MOTOR_NUM):
            await self.set_motor_ctrl_param(i, 0, 0)
            # await self.set_motor_enc_param(i, 1, 1)
            # await self.set_motor_param(i, 1, 4599, 1)
        await asyncio.sleep(0.1)
        
        self.id_yaw_ = [1,2]
        # 运动至零位,其中小拇指和无名指需要上电前复位
        for i in range(MOTOR_NUM):
            if i in self.id_yaw_:
                pass
            else:
                await self.set_motor_target(i, 0, 0, 500*self.motor_direction_["all"][i])
        await asyncio.sleep(0.5)
        for i in range(MOTOR_NUM):
            if i in self.id_yaw_:
                pass
            # elif i == 12 or i == 6 or i == 7:
            #     await self.set_motor_target(i, 0, 0, -800*self.motor_direction_["all"][i])
            else:
                await self.set_motor_target(i, 0, 0, -1000*self.motor_direction_["all"][i])
            # elif i == 5 or i == 6 or i == 10:
            #     await self.set_motor_target(i, 0, 0, -3000*self.motor_direction_["all"][i])
        await asyncio.sleep(4)
        logger.info("回到零位")
                
        # 清零编码器
        for i in range(MOTOR_NUM):
            if i in self.id_yaw_:
                pass
            else:
                await self.clear_enc(i)
        await asyncio.sleep(0.1)
        logger.info("清零编码器")
        
        # 电机控制初始参数
        if self.mode_ == 'p_p':
            for i in range(MOTOR_NUM): 
                await self.set_motor_ctrl_param(i, self.motor_kp_[i], self.motor_kd_[i])
                # await self.set_motor_enc_param(i, self.enc_dir_[i], 1)

        elif self.mode_ == 'p_t':
            for i in range(MOTOR_NUM): 
                await self.set_motor_ctrl_param(i, 0, 0)
        
        # 异步线程-始终获取电机状态
        await self.reset_id()
        self.enable_rx_ = 1
        await asyncio.sleep(0.5)
        self.recv_task_ = asyncio.create_task(self.read_forever())
        
        logger.info("开始读取")
        
        await self.reset_id()
        await asyncio.sleep(2)
        self.update_task_ = asyncio.create_task(self.update_data())
        await asyncio.sleep(1)
        
        if self.mode_ == 'p_t':
            logger.info("使用力控模式时, 添加控制异步任务self.send_cmds()")
            self.send_task_ = asyncio.create_task(self.send_cmds())
            await asyncio.sleep(1)
        
        logger.info("初始化完成")
            
    async def set_queue(self, fg_name):
        # i = self.hand_model_.finger_mapping[fg_name]-1
        # while True:
        #     if not self.queue_[i].empty():
        #         q = self.queue_[i].get()  # 从队列获取命令
        #         # logger.info(f"{fg_name} i: {i} q: {q} - shape: {q.shape[0]}")
        #         await self.set_q(q, fg_name=fg_name)
        #     else:
        #         await asyncio.sleep(0.5)
        while True:
            if fg_name == "index":
                if not self.queue_index_.empty():
                    q = self.queue_index_.get()  # 从队列获取命令
                    await self.set_q(q, fg_name=fg_name)
                else:
                    await asyncio.sleep(0.5)
            elif fg_name == "middle":
                if not self.queue_middle_.empty():
                    q = self.queue_middle_.get()  # 从队列获取命令
                    await self.set_q(q, fg_name=fg_name)
                else:
                    await asyncio.sleep(0.5)
            elif fg_name == "fourth":
                if not self.queue_fourth_.empty():
                    q = self.queue_fourth_.get()  # 从队列获取命令
                    await self.set_q(q, fg_name=fg_name)
                else:
                    await asyncio.sleep(0.5)
            elif fg_name == "little":
                if not self.queue_little_.empty():
                    q = self.queue_little_.get()  # 从队列获取命令
                    await self.set_q(q, fg_name=fg_name)
                else:
                    await asyncio.sleep(0.5)
            elif fg_name == "thumb":
                if not self.queue_thumb_.empty():
                    q = self.queue_thumb_.get()  # 从队列获取命令
                    await self.set_q(q, fg_name=fg_name)
                else:
                    await asyncio.sleep(0.5)
            
            # if not self.queue_[i].empty():
            #     q = self.queue_[i].get()  # 从队列获取命令
            #     # logger.info(f"{fg_name} i: {i} q: {q} - shape: {q.shape[0]}")
            #     await self.set_q(q, fg_name=fg_name)
            # else:
            #     await asyncio.sleep(0.5)
                
    def add_queue(self, q, fg_name):
        # i = self.hand_model_.finger_mapping[fg_name]-1
        # self.queue_[i].put(q)
        # logger.info(f"{fg_name} i: {i} q: {q} - shape: {q.shape[0]}")
        
        if fg_name == "index":
            self.queue_index_.put(q)
        elif fg_name == "middle":
            self.queue_middle_.put(q)
        elif fg_name == "fourth":
            self.queue_fourth_.put(q)
        elif fg_name == "little":
            self.queue_little_.put(q)
        elif fg_name == "thumb":
            self.queue_thumb_.put(q)

    async def run(self):
        await self.init()
        
        logger.info("进入run()")
        # 始终发送队列
        asyncio.create_task(self.set_queue("index"))
        asyncio.create_task(self.set_queue("middle"))
        asyncio.create_task(self.set_queue("fourth"))
        asyncio.create_task(self.set_queue("little"))
        asyncio.create_task(self.set_queue("thumb"))
        
        while True:
            await asyncio.sleep(0.1)

        logger.info("退出run()")
    # ############################ 用户接口 ############################
    
    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(self.target_ip_, self.target_port_)
        
    async def send_cmd(self, cmd : CommPack):
        rawdata = cmd.pack()
        await self.tf.send_async(cmd.pkt_type, rawdata)
        time.sleep(0.002)
        # await asyncio.sleep(0.0002)
        await asyncio.sleep(0.01)
        
    async def tf_write_async(self, data: bytes) -> int:
        self.writer.write(data)
        await self.writer.drain()
        
    def tf_general_listener(self, tf, frame:TF.TF_Msg):
        self.rx_cnt += 1
        # logger.info(f"recv type={frame.type}, len={frame.len}, cnt={self.rx_cnt}")
        if frame.type == COMM_GET_MOTOR_STATE:
            try:
                motor_states_reply = ReplyGetMotorStates(frame.data)
                # os.system("cls")
                # motor_states_reply.print()
                for i, motor_state in enumerate(motor_states_reply.motor_states):
                    id, enc, output = motor_state
                    # print("Motor State {}: id={}, enc={}, output={}".format(i, id, enc, output))
                    
                    # 由于目前版本电机正反转存在问题
                    # self.data_.qd[id] = 0.5 * self.data_.qd[id] + 0.5 * (enc*self.motor_direction_["all"][id] - self.data_.q[id])/0.002
                    # self.data_.q[id] = enc*self.motor_direction_["all"][id]
                    # self.data_.cur[id] = output*self.motor_direction_["all"][id]
                    self.data_.q[id] = enc
                if self.rx_cnt % 30 == 0:
                    # logger.info(f"data: {self.data_.q}")
                    # logger.info(f"vel : {self.data_.qd}")
                    logger.info(f"获取data: {self.data_.q}")
            except Exception as e:
                tb = traceback.print_exc()
    
    async def reset_id(self):
        pkt = CommIDSet(0)
        await self.send_cmd(pkt)
        
    async def set_motor_ctrl_param(self, motor_id, kp, kd):
        pkt = CommMotorCtrlParam(motor_id, kp, kd)
        await self.send_cmd(pkt)

    async def set_motor_enc_param(self, motor_id, cw_counting_dir, cnt2rad_ratio):
        pkt = CommMotorEncParam(motor_id, cw_counting_dir, cnt2rad_ratio)
        await self.send_cmd(pkt)
        
    async def clear_enc(self, motor_id):
        pkt = CommClearEnc(motor_id)
        await self.send_cmd(pkt)
        
    async def set_motor_param(self, motor_id, min_pwm, max_pwm, cw_dir_level):
        pkt = CommMotorParam(motor_id, min_pwm, max_pwm, cw_dir_level)
        await self.send_cmd(pkt)

    async def set_motor_target(self, motor_id, dest_pos, dest_vel, dest_torque):
        pkt = CommMotorTarget(motor_id, dest_pos, dest_vel, dest_torque)
        await self.send_cmd(pkt)
    
    async def read_forever(self):
        while self.enable_rx_:
            data = await self.reader.read(1024)
            self.tf.accept(data)
    
    async def check_alive(self):
        pkt = CommAliveCheck()
        await self.send_cmd(pkt)
    
    async def get_motor_states(self):
        pkt = CommGetMotorStates()
        await self.send_cmd(pkt)
    
    async def update_data(self):
        while True:
            await self.get_motor_states()
            await asyncio.sleep(1)
            
    async def send_cmds(self):
        # self.cmd_始终为当前位置速度指令
        while True:
            pass
            # # logger.info(f"正在线程{threading.current_thread()}中发送cmd_ : {self.cmd_.qDes}")
            # if self.mode_ == 'p_p':
            #     logger.error("send_cmds()只在力控模式下使用")
                
            # elif self.mode_ == 'p_t':
            #     for i in range(MOTOR_NUM):
            #         await self.set_motor_ctrl_param(i, 0, 0)
                    
            #     d_now = await self.get_pos_motors()
            #     dd_now = await self.get_vel_motors()
                
            #     self.error_sum += (self.cmd_pt_.qDes - d_now)
            #     # logger.info(f"误差累积 : {self.error_sum}")
            #     # logger.info(f"正在线程{threading.current_thread()}中发送tau_ : {tau}")           
            #     tau = self.motor_kp_ * (self.cmd_pt_.qDes - d_now) + \
            #           self.motor_kd_ * (self.cmd_pt_.qdDes - dd_now) + \
            #           self.motor_ki_ * self.error_sum
                      
            #     for i in range(MOTOR_NUM):   
            #         await self.set_motor_target(i, 
            #                                     0, 
            #                                     0, 
            #                                     tau[i]*self.motor_dir_[i])
      
    async def stop(self):                        
        for i in range(MOTOR_NUM):
            await self.set_motor_ctrl_param(i, 0, 0)
        await asyncio.sleep(0.1)
        
        # 异步事件循环已经结束，不需要手动关闭
        self.recv_task_.cancel()
        self.update_task_.cancel()
        if self.mode_ == 'p_t':
            self.send_task_.cancel()
    
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()          
        
    async def close(self):
        await self.stop()
        logger.info("关闭...")  
            
    def map_motor(self, x, name):
        # 映射电机指令
        # motor_map = [9,8,7,5,10,6,2,4,1,3,0,11,12]
        x_map = np.zeros_like(x)
        mapping = self.motor_mapping_[name]
        ratios = ratio[name]
        direction = self.motor_direction_[name]
            
        for i, (idx, r, dir) in enumerate(zip(mapping, ratios, direction)):
            x_map[i] = x[i] * r * dir
                
        return x_map, mapping
    
    async def set_d(self, q, qd, kp, kd):
        # 若为位置模式，则直接发送；若为力控模式，保存于cmd_pt_
        if self.mode_ == 'p_p':
            pass
        elif self.mode_ == 'p_t':
            self.cmd_pt_.qDes = q
            self.cmd_pt_.qdDes = qd
            self.cmd_pt_.kpDes = kp
            self.cmd_pt_.kdDes = kd
            self.cmd_pt_.tauDes = np.zeros(MOTOR_NUM)

    async def set_q(self, q, dt=1, n=30, fg_name=None):
        q = np.asarray(q)

        if self.plan_ == 0:
            if fg_name is not None:
                # 设置单个手指运动, 映射，然后发送
                d = self.hand_model_.ik_q2d(q, fg_name)
                d_map, mapping = self.map_motor(d, fg_name)
            else:
                logger.error("未指定手指名称")
                
            for i in range(d.shape[0]):
                await self.set_motor_target(mapping[i], d_map[i], 0, 0)
            # await asyncio.sleep(dt - d.shape[0]*0.004)
            await asyncio.sleep(dt)
            logger.info(f"设置{fg_name}的电机位置为: {d_map}")
                
        elif self.plan_ == 1:
            t0 = 0
            tf = t0 + dt
            
            if fg_name is not None:
                d_now = await self.get_d(fg_name)
                
                d = self.hand_model_.ik_q2d(q, fg_name)
                d_map, mapping = self.map_motor(d, fg_name)                
            else:
                logger.error("未指定手指名称")
                
            d_traj, _ = self.hand_model_.plan_in_dspace_all(t0,d_now,np.zeros_like(d),np.zeros_like(d),
                                                                tf,d_map,    np.zeros_like(d),np.zeros_like(d),
                                                                n)
            for i in range(d_traj.shape[1]):
                for id in range(d.shape[0]):                
                    await self.set_motor_target(mapping[id], d_traj[id,i], 0, 0)
                # logger.info(f"设置{fg_name}的电机位置为: {d_traj[:,i]}")
                await asyncio.sleep(dt/n - d.shape[0]*0.004)  
            logger.info(f"设置{fg_name}的电机位置为: {d_traj[:,-1]}") 
    
    async def get_dd(self):
        # 此处是编码器pos
        # with self.lock_:
        dd = self.data_.qd
        return dd
    
    async def get_d(self, fg_name = None):
        # 此处是编码器pos
        if fg_name is not None:
            mapping = self.motor_mapping_[fg_name]
            d = [self.data_.q[i] for i in mapping]
            return np.array(d)
            
        else:
            d = self.data_.q
            return d
    
    async def get_q(self):
        # TODO: 用于显示，需要正解,存在问题,
        pass
    #     d = self.get_pos_motors()
    #     # q = self.model.fk_d2d_all(d)
    #     # return q
    
    async def action(self, action_name, dt=0.5, n=30):
        logger.info('='*15 + ' action: ' + action_name + ' ' + '='*15)
        action = self.actions[action_name]
        
        # tasks = [self.set_q(action[i], dt, n, finger[0]) for i, finger in enumerate(zip(finger_name))]
        # await asyncio.gather(*tasks)

        tasks = []
        for i, (finger) in enumerate(zip(finger_name)):
            # if finger[0] == "thumb":
            #     pass
            # else:
            tasks.append(self.set_q(action[i], dt, n, finger[0]))
            # asyncio.create_task(self.set_q(action[i], dt, n, finger[0]))
        await asyncio.gather(*tasks)
        
        await asyncio.sleep(dt)
        
    async def demo(self):
        await self.init() 
        
        # t = time.time()
        # while time.time() -t < 10:
        #     for i in range(MOTOR_NUM):
        #         if i in [6,7,8,9]:
        #             await self.set_motor_target(i, 2000*self.motor_direction_['all'][i],0,0)  # 如果这不await呢
        #         # await asyncio.sleep(0.1)
        #     await asyncio.sleep(0.3)
            
        #     for i in range(MOTOR_NUM):
        #         if i in [6,7,8,9]:
        #             await self.set_motor_target(i,0,0,0)
        #     await asyncio.sleep(0.3)
        
        # while(1):
        #     await asyncio.sleep(1)
        
        # async def index_move(q1,q2, dt, n, n_loop=1):
        #     for _ in range(n_loop):
        #         await self.set_q(np.array([0,q1,q2]), n=n, dt=dt, fg_name="index")
        #         await self.set_q(np.array([0,0,0]), n=n, dt=dt, fg_name="index")
        # async def middle_move(q1,q2, dt, n, n_loop=1):
        #     for _ in range(n_loop):
        #         await self.set_q(np.array([0,q1,q2]), n=n, dt=dt, fg_name="middle")
        #         await self.set_q(np.array([0,0,0]), n=n, dt=dt, fg_name="middle")
        # async def fourth_move(q, dt, n, n_loop=1):
        #     for _ in range(n_loop):
        #         await self.set_q(np.array([0,q]), n=n, dt=dt, fg_name="fourth")
        #         await self.set_q(np.array([0,0]), n=n, dt=dt, fg_name="fourth")
        # async def little_move(q, dt, n, n_loop=1):
        #     for _ in range(n_loop):
        #         await self.set_q(np.array([0,q]), n=n, dt=dt, fg_name="little")
        #         await self.set_q(np.array([0,0]), n=n, dt=dt, fg_name="little")
        # async def thumb_move(q1,q2, dt, n, n_loop=1):
        #     for _ in range(n_loop):
        #         await self.set_q(np.array([0,q1,q2]), n=n, dt=dt, fg_name="thumb")
        #         await self.set_q(np.array([0,0,0]), n=n, dt=dt, fg_name="thumb")
                
        # logger.info("弯曲演示")
        # q = 1.45
        # dt = 0.8
        # n = 30
        # for _ in range(1):
        #     # await index_move(q, 0.0, dt, n)
        #     # await index_move(0.0, q, dt, n)
        #     # await middle_move(q, 0.0, dt, n)
        #     # await middle_move(0.0, q, dt, n)
            
        #     # await index_move(q, q, dt, n)
        #     # await middle_move(q, q, dt, n)
        #     # await fourth_move(q, dt, n)
        #     # await little_move(q, dt, n)
        #     # await thumb_move(q, 0.0, dt, n)
        #     # await thumb_move(0.0, q, dt, n)
        #     # await thumb_move(q, q, dt, n)
            
        #     await little_move(q, dt, n)
        #     await fourth_move(q, dt, n)
        #     await middle_move(q, 0.0, dt, n)
        #     await middle_move(0.0, q, dt, n)
        #     await middle_move(q, q, dt, n)
        #     await index_move(q, 0.0, dt, n)
        #     await index_move(0.0, q, dt, n)
        #     await index_move(q, q, dt, n)
        #     await thumb_move(q, 0.0, dt, n)
        #     await thumb_move(0.0, q, dt, n)
        #     await thumb_move(q, q, dt, n)
                
        # logger.info("侧摆演示")

        
        # dt = 0.8
        # n = 30
        # for name in ["index", "middle"]:
        #     await self.set_q(np.array([0.3,0.3,0]), n=n, dt=dt, fg_name=name)
        #     # await asyncio.sleep(dt)
        # for name in ["fourth", "little"]:
        #     await self.set_q(np.array([0.3,0.0]), n=n, dt=dt, fg_name=name)
        # for name in reversed(["fourth", "little"]):
        #     await self.set_q(np.array([-0.3,0.0]), n=n, dt=dt, fg_name=name)
        # for name in reversed(["index", "middle"]):
        #     await self.set_q(np.array([-0.3,0.3,0]), n=n, dt=dt, fg_name=name)
        # await asyncio.sleep(dt)
        
        # await self.action('diverge', dt=0.5, n=30)
        # await self.action('open', dt=0.5, n=30)
        
        # logger.info("动作演示")
        # for _ in range(1):
        #     await self.action('one')
        #     await self.action('two')
        #     await self.action('three')
        #     await self.action('four')
        #     await self.action('five')
        #     await self.action('six')
        #     await self.action('seven')
        #     await self.action('eight')
        #     await self.action('nine')

        # await self.action('ok')
        # await self.action('open')
        # await asyncio.sleep(1)
        
        # logger.info("异步演示")
        # self.plan_ = 1
        # q = 1.5
        # dt = 0.8
        # dt2 = 0.2
        # n = 30
        # n_loop = 3
        # for _ in range(1):
        #     asyncio.create_task(index_move(q, q, dt, n, n_loop))
        #     await asyncio.sleep(dt2)
        #     asyncio.create_task(middle_move(q, q, dt, n, n_loop))
        #     await asyncio.sleep(dt2)
        #     asyncio.create_task(fourth_move(q, dt, n, n_loop))
        #     await asyncio.sleep(dt2)
        #     asyncio.create_task(little_move(q, dt, n, n_loop))
        #     await asyncio.sleep(dt2)
        
        while 1:
            await asyncio.sleep(0.1)
           
async def main():
    hand_path = '/home/ccs/test_ws/src/freehand_controller/freehand_controller/individual/description/urdf/freehand_right.urdf'
    hand = HandAPI(hand_path)
    
    joints = np.array([0,0,0])
    d = hand.hand_model_.ik_q2d(joints, "thumb")
    logger.info(hand.map_motor(d, "thumb"))
    
    await hand.demo()
    await asyncio.sleep(1)
    
    # logger.info("异步演示")
    # q = 1.45
    # dt = 0.8
    # dt2 = 0.2
    # n = 30
    # n_loop = 3
    # for _ in range(1):
    #     asyncio.create_task(index_move(q, q, dt, n, n_loop))
    #     await asyncio.sleep(dt2)
    #     asyncio.create_task(middle_move(q, q, dt, n, n_loop))
    #     await asyncio.sleep(dt2)
    #     asyncio.create_task(fourth_move(q, dt, n, n_loop))
    #     await asyncio.sleep(dt2)
    #     asyncio.create_task(little_move(q, dt, n, n_loop))
    #     await asyncio.sleep(dt2)
    
    while 1:
        await asyncio.sleep(0.1)
    
    await hand.close()


if __name__ == '__main__':
    asyncio.run(main())
    