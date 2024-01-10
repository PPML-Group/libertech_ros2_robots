import sys
import os
import TinyFrame as TF
from CommProtocol import *
import socket
import time
import asyncio
import traceback
from rich.traceback import Traceback

# TARGET_IP = '10.10.20.61'
TARGET_IP = '192.168.1.200'
TARGET_PORT = 2000
# TARGET_IP = '127.0.0.1'
# TARGET_PORT = 8801

class FlexHand:
    def __init__(self, reader:asyncio.StreamReader, writer:asyncio.StreamWriter):
        self.tf = TF.TinyFrame()
        self.tf.SOF_BYTE = 0x01
        self.tf.ID_BYTES = 0x01
        self.tf.LEN_BYTES = 0x02
        self.tf.TYPE_BYTES = 0x01
        self.tf.CKSUM_TYPE = 'crc16'
        
        self.tf.write = self.tf
        self.tf.write_async = self.tf_write_async
        self.tf.add_fallback_listener(self.tf_general_listener)
        
        self.reader = reader
        self.writer = writer
        
        self.enable_rx = True
        self.rx_cnt = 0
    
    async def send_cmd(self, cmd : CommPack):
        rawdata = cmd.pack()
        await self.tf.send_async(cmd.pkt_type, rawdata)
        # print("s")
        time.sleep(0.001)
        await asyncio.sleep(0.001)
        
    async def tf_write_async(self, data: bytes) -> int:
        self.writer.write(data)
        await self.writer.drain()
        
    def tf_general_listener(self, tf, frame:TF.TF_Msg):
        self.rx_cnt += 1
        print(f"recv type={frame.type}, len={frame.len}, cnt={self.rx_cnt}")
        if frame.type == COMM_GET_MOTOR_STATE:
            try:
                motor_states_reply = ReplyGetMotorStates(frame.data)
                # os.system("cls")
                motor_states_reply.print()
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
        while self.enable_rx:
            data = await self.reader.read(1024)
            self.tf.accept(data)
    
    async def check_alive(self):
        pkt = CommAliveCheck()
        await self.send_cmd(pkt)
    
    async def get_motor_states(self):
        pkt = CommGetMotorStates()
        await self.send_cmd(pkt)
    
    async def update_motor_states(self):
        while True:
            await self.get_motor_states()
            await asyncio.sleep(0.1)

async def user_task(flexhand:FlexHand):
    await flexhand.reset_id()

    await asyncio.sleep(1)
    update_task = asyncio.create_task(flexhand.update_motor_states())
    await asyncio.sleep(1)
    
    enc_dir_   = [1, 0, 0, 0, 1, 0, 0, 0, 1, 1,  1,  0,  0]
    motor_dir_ = [0, 1, 1, 1, 1, 1, 1, 1, 1, 1,  1,  1,  1]
    
    kp =   [1, 1, 1, 1, 1, 1, 1, 2, 1, 1,  1,  1,  1]
    
    dir_ = [-1, 1, 1, 1, 1, 1,-1,-1, 1, 1, 1, -1, 1]
    dir_ = [ 1,-1,-1, 1, 1, 1,-1,-1, 1, 1, 1, -1, 1]
    for i in range(13):
        # await flexhand.set_motor_ctrl_param(i,0,0)
        await flexhand.set_motor_ctrl_param(i,1,0) 
        
    # for i in range(13):
    #     await flexhand.set_motor_param(i, 0, 2799, motor_dir_[i])
    #     await asyncio.sleep(0.5)
    
        # time.sleep(0.3)
        # await flexhand.set_motor_enc_param(i, enc_dir_[i], 1)
        # await asyncio.sleep(0.3)
    
    print("*"*10)
    
    test_id = 8
    
    # await flexhand.set_motor_param(test_id, 0, 1, motor_dir_[test_id])
    # await flexhand.set_motor_enc_param(test_id, enc_dir_[test_id], 1)
    
    t = time.time()
    while time.time() -t < 10:
        for i in range(13):
            if i == test_id:
                await flexhand.set_motor_target(i, 2000*dir_[i],0,0)  # 如果这不await呢
            # await asyncio.sleep(0.1)
        await asyncio.sleep(0.3)
        
        # await flexhand.set_motor_target(test_id,5000*dir_[i],0,0)
        # await asyncio.sleep(0.3)
        
        # for i in range(13):
        #     # if i == test_id:
        #     await flexhand.set_motor_target(i,0*dir_[i],0,0)
        # await asyncio.sleep(0.3)

        # await flexhand.set_motor_target(test_id, 800*dir_[test_id],0,0)  # 如果这不await呢
        # await asyncio.sleep(0.3)
        
        # await flexhand.set_motor_target(test_id,-800,0,0)
        # await asyncio.sleep(0.3)

    
        
async def main():
    reader, writer = await asyncio.open_connection(TARGET_IP, TARGET_PORT)
    
    flexhand = FlexHand(reader, writer)
    
    # 启动接收
    recv_task = asyncio.create_task(flexhand.read_forever())
    
    await user_task(flexhand)
    
    recv_task.cancel()
    
    writer.close()
    await writer.wait_closed()


if __name__ == '__main__':
    asyncio.run(main())