from rich.logging import RichHandler
import logging
import os

current_directory = os.path.dirname(os.path.abspath(__file__))
log_file_path = os.path.join(current_directory, 'freehand.log')

logger = logging.getLogger(__name__)
logger.setLevel(level=logging.INFO)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler = logging.FileHandler(log_file_path)
handler.setLevel(logging.INFO)
handler.setFormatter(formatter)

rich_handler = RichHandler(
    show_time=False,  # 显示时间
    show_level=True,  # 显示日志级别
    rich_tracebacks=True  # 使用 Rich 格式显示 traceback
)
rich_handler.setFormatter(formatter)
rich_handler.setLevel(logging.DEBUG)
# console = logging.StreamHandler()
# console.setLevel(logging.INFO)
# console.setFormatter(formatter)
logger.addHandler(handler)
logger.addHandler(rich_handler)
# logger.addHandler(console)
logger.info("Start log")
