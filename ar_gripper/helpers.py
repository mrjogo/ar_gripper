import logging


class ConnectPythonLoggingToROS(logging.Handler):
    def __init__(self, node_logger):
        super().__init__()
        self.MAP = {
            logging.DEBUG: node_logger.debug,
            logging.INFO: node_logger.info,
            logging.WARNING: node_logger.warning,
            logging.ERROR: node_logger.error,
            logging.CRITICAL: node_logger.fatal,
        }

    def emit(self, record):
        try:
            self.MAP[record.levelno](f"{record.name}: {record.msg}")
        except KeyError:
            self.MAP[logging.ERROR](
                f"unknown log level {record.levelno} LOG: "
                f"{record.name}: {record.msg}"
            )
        except Exception:
            print("Exception raised while trying to emit Python log to ROS logger: ")
            print(logging.traceback.format_exc())
