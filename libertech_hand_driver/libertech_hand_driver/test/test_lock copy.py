import asyncio
import threading

class YourClass:
    def __init__(self):
        self.cmd_lock = asyncio.Lock()
        self.cmd_ = "your_command_here"
        # 其他初始化代码...
        
        self.thread_ = None

    async def run(self):
        # your existing async tasks here...
        self.task = asyncio.create_task(self.task())
        
        while 1:            
            pass

    def run_loop(self):
        
        asyncio.run(self.run())
        
    async def task(self):
        while 1: 
            async with self.cmd_lock:
                print(f"我正在线程{threading.current_thread()}中发送cmd_ : {self.cmd_}")
    


    async def set_cmd(self, new_cmd):
        async with self.cmd_lock:
            self.cmd_ = new_cmd

    async def get_cmd(self):
        async with self.cmd_lock:
            return self.cmd_
        
# 主线程中设置 self.cmd_
async def main():
    your_object = YourClass()
    # 启动异步任务管理线程
    your_object.thread_ = threading.Thread(target=your_object.run_loop, daemon=True)
    your_object.thread_.start()

    # 在主线程中设置 self.cmd_
    cnt = 0
    while 1:  
        new_cmd_value = cnt
        await your_object.set_cmd(new_cmd_value)
        print(f"我正在线程{threading.current_thread()}中设置cmd_ : {cnt}")
        cnt += 1

if __name__ == '__main__':
    asyncio.run(main())
    
    
# # 主线程中获取 self.cmd_
# async def another_function():
#     your_object = YourClass()
#     cmd_value = await your_object.get_cmd()
#     print(cmd_value)  # 使用获取到的 cmd 值做其他操作

    # def task(self):
    #     while 1: 
    #         print(f"我正在线程{threading.current_thread()}中发送cmd_ : {self.cmd_}")