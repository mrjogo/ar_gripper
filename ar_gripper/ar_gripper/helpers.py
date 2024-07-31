import logging


class ConnectPythonLoggingToROS(logging.Handler):
    def __init__(self, node_logger):
        super().__init__()
        self.node_logger = node_logger

    def emit(self, record):
        try:
            msg = f"{record.name}: {record.msg}"
            # Each ROS log level call must be on a different line or you get the
            # exception "ValueError: Logger severity cannot be changed between calls."
            if record.levelno == logging.DEBUG:
                self.node_logger.debug(msg)
            elif record.levelno == logging.INFO:
                self.node_logger.info(msg)
            elif record.levelno == logging.WARNING:
                self.node_logger.warning(msg)
            elif record.levelno == logging.ERROR:
                self.node_logger.error(msg)
            elif record.levelno == logging.CRITICAL:
                self.node_logger.fatal(msg)
            else:
                self.node_logger.warn(
                    f"unknown log level {record.levelno} LOG: "
                    f"{record.name}: {record.msg}"
                )

        except Exception:
            print("Exception raised while trying to emit Python log to ROS logger: ")
            print(logging.traceback.format_exc())
