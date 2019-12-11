import rospy


def format_message(msg):
    name = rospy.get_name()
    if name.startswith('/'):
        name = name[1:]
    name = name.replace('/', '-')
    name = name.replace('_', '-')
    return "[{}] {}".format(name, msg)


def log_info(msg, *args, **kwargs):
    rospy.loginfo(format_message(msg), *args, **kwargs)


def log_debug(msg, *args, **kwargs):
    rospy.logdebug(format_message(msg), *args, **kwargs)


def log_warn(msg, *args, **kwargs):
    rospy.logwarn(format_message(msg), *args, **kwargs)


def log_error(msg, *args, **kwargs):
    rospy.logerr(format_message(msg), *args, **kwargs)
