import logging
import rospy


class ConnectPythonLoggingToROS(logging.Handler):

    MAP = {
        logging.DEBUG: rospy.logdebug,
        logging.INFO: rospy.loginfo,
        logging.WARNING: rospy.logwarn,
        logging.ERROR: rospy.logerr,
        logging.CRITICAL: rospy.logfatal,
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno](f"{record.name}: {record.msg}")
        except KeyError:
            rospy.logerr(
                f"unknown log level {record.levelno} LOG: "
                f"{record.name}: {record.msg}"
            )
