import logging
import os
import time

# Set the log format and level using basicConfig
logging.basicConfig(
    format='%(asctime)s.%(msecs)03d | %(name)s | %(levelname)s | %(message)s |',
    datefmt='%m-%d-%Y %H:%M:%S',
    level=logging.DEBUG
)

# Create a file handler
if not os.path.exists("logs"):
    os.mkdir("logs")
timestr = time.strftime("%Y%m%d-%H%M%S")
file_handler = logging.FileHandler(f"logs/fourth_stage_{timestr}.log")
file_handler.setLevel(logging.DEBUG)

# Create a console handler
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

# Create a formatter for both console and file
formatter = logging.Formatter('%(asctime)s.%(msecs)03d | %(name)s | %(levelname)s | %(message)s |', datefmt='%m-%d-%Y %H:%M:%S')

# Set the formatter for the file handler
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

# Get the root logger
root_logger = logging.getLogger()

# Add the file handler to the root logger
root_logger.addHandler(file_handler)
# root_logger.addHandler(console_handler)



# Old logging file. Reformatted to above new ones.
# import time
# import logging
# import sys
# import os

# def setup_logger():
#     timestr = time.strftime("%Y%m%d-%H%M%S")
#     logger = logging.getLogger()
#     logger.setLevel(logging.DEBUG)
#     formatter = logging.Formatter('%(asctime)s.%(msecs)03d | %(levelname)s | %(message)s','%m-%d-%Y %H:%M:%S')
#     stdout_handler = logging.StreamHandler(sys.stdout)
#     stdout_handler.setLevel(logging.DEBUG)
#     stdout_handler.setFormatter(formatter)
#     if not os.path.exists("logs"):
#         os.mkdir("logs")
#     file_handler = logging.FileHandler(
#         "logs/third_stage_{}.log".format(timestr))
#     file_handler.setLevel(logging.DEBUG)
#     file_handler.setFormatter(formatter)
#     logger.addHandler(file_handler)
#     logger.addHandler(stdout_handler)
#     return logger
