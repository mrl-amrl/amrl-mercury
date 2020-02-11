import rospy
from os.path import basename
from inspect import getframeinfo, stack

_stack_print = False


def stack_print(status):
    global _stack_print
    _stack_print = status


def format_message(msg):
    name = rospy.get_name()
    if name.startswith('/'):
        name = name[1:]
    name = name.replace('/', '-')
    name = name.replace('_', '-')
    if _stack_print:
        caller = getframeinfo(stack()[2][0])
        caller = "{}:{}".format(basename(caller.filename), caller.lineno)
        return "[{}] [{}] {}".format(name, caller, msg)
    return "[{}] {}".format(name, msg)


def log_info(msg, *args, **kwargs):
    rospy.loginfo(format_message(msg), *args, **kwargs)


def log_debug(msg, *args, **kwargs):
    rospy.logdebug(format_message(msg), *args, **kwargs)


def log_warn(msg, *args, **kwargs):
    rospy.logwarn(format_message(msg), *args, **kwargs)


def log_error(msg, *args, **kwargs):
    rospy.logerr(format_message(msg), *args, **kwargs)
