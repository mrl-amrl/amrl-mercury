import shlex
from subprocess import Popen, PIPE, STDOUT


def get_simple_cmd_output(cmd, stderr=STDOUT):
    args = shlex.split(cmd)
    return Popen(args, stdout=PIPE, stderr=stderr).communicate()[0]


def get_ping_time(host):
    host = host.split(':')[0]
    cmd = "fping {host} -C 3 -q".format(host=host)
    result = str(get_simple_cmd_output(cmd)).replace('\\', '').split(':')[-1].replace("n'", '').replace("-",
                                                                                                        '').replace(
        "b''", '').split()
    res = [float(x) for x in result]
    if len(res) > 0:
        return sum(res) / len(res)
    return 999
