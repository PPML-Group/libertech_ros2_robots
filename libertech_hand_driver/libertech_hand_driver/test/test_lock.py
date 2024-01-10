import asyncio
import threading
import numpy as np

# 如果把cmd换为numpy可以，换为class呢

class FingerCommand:
    def __init__(self):
        self.zero()

    def zero(self):    
        self.qDes = np.zeros(5)

class YourClass:
    def __init__(self):
        # self.cmd_lock = asyncio.Lock()
        self.cmd_ = FingerCommand()
        # 其他初始化代码...
        
        # self.thread_ = None
        # 启动异步任务管理线程
        self.thread_ = threading.Thread(target=self.run_loop, daemon=True)
        self.thread_.start()

    async def run(self):
        # your existing async tasks here...
        self.task_ = asyncio.create_task(self.async_task())
        await asyncio.sleep(0.5)
        
        while 1:            
            # async with self.cmd_lock:
            # 你的操作
                # pass
            await asyncio.sleep(0.1)  # 等待一小段时间，释放事件循环控制权

    def run_loop(self):
        # loop = asyncio.new_event_loop()
        # asyncio.set_event_loop(loop)
        # loop.run_until_complete(self.run())
        asyncio.run(self.run())
        
    async def async_task(self):
        while 1: 
            # async with self.cmd_lock:
            print(f"我正在线程{threading.current_thread()}中发送cmd_ : {self.cmd_.qDes}")
            await asyncio.sleep(0.1)
    # def task(self):
    #     while 1: 
    #         print(f"我正在线程{threading.current_thread()}中发送cmd_ : {self.cmd_}")


    async def set_cmd(self, new_cmd):
        # async with self.cmd_lock:
        self.cmd_.qDes = new_cmd

    # async def get_cmd(self):
    #     async with self.cmd_lock:
    #         return self.cmd_
        
# 主线程中设置 self.cmd_
async def main():
    your_object = YourClass()
    

    # 在主线程中设置 self.cmd_
    cnt = 0
    while 1:  
        new_cmd_value = cnt * np.ones(5)
        await your_object.set_cmd(new_cmd_value)
        print(f"我正在线程{threading.current_thread()}中设置cmd_ : {new_cmd_value}")
        cnt += 1
        await asyncio.sleep(0.1)

if __name__ == '__main__':
    asyncio.run(main())
    
    
    
# # 主线程中获取 self.cmd_
# async def another_function():
#     your_object = YourClass()
#     cmd_value = await your_object.get_cmd()
#     print(cmd_value)  # 使用获取到的 cmd 值做其他操作
