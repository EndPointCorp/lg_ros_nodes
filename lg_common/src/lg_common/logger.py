import logging
import os
import sys
import rospy
from logging.handlers import TimedRotatingFileHandler
FORMATTER = logging.Formatter('[%(asctime)s] p%(process)s %(name)s - {%(pathname)s:%(lineno)d} %(levelname)s - %(message)s', '%m-%d %H:%M:%S')
LOG_FILE = "my_app.log"


def get_console_handler():
    console_handler = logging.StreamHandler(sys.stderr)
    console_handler.setFormatter(FORMATTER)
    return console_handler


def get_file_handler(log_file=LOG_FILE):
    file_handler = TimedRotatingFileHandler(log_file, when='midnight')
    file_handler.setFormatter(FORMATTER)
    return file_handler


def get_logger(logger_name, log_level=logging.INFO):
    """Get a logger.

    Use log_level arg unless VPROS_LOG_LEVEL environment variable is set."""
    name = rospy.get_name() + ':' + logger_name
    logger = logging.getLogger(name)
    if (level_env := os.environ.get("VPROS_LOG_LEVEL", None)) is not None:
        # TODO: Python 3.11+ should use getLevelNamesMapping
        log_level = logging.getLevelName(level_env.upper())
    logger.setLevel(log_level)
    logger.addHandler(get_console_handler())
    logger.addHandler(get_file_handler(log_file=logger_name + ".log"))
    # TODO add file handler with some path in /home/lg/.ros/log/latest/<logger_name>.log
    # with this pattern, it's rarely necessary to propagate the error up to parent
    logger.propagate = False
    return logger

