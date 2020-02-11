import rospy
import yaml
import os

from mercury import logger
from mercury_dynparam.srv import Load, Default
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from dynamic_reconfigure import DynamicReconfigureCallbackException, DynamicReconfigureParameterException


class DynamicConfigurationParameters:
    def __init__(self):
        self.defaults = {}
        for param_name in rospy.get_param_names():
            if not param_name.startswith(rospy.get_name()):
                continue
            value = str(rospy.get_param(param_name))
            index = value.find(';')
            name = value[:index]
            path = value[index+1:]
            param_name = param_name.split('/')[2]
            self.defaults[param_name] = {
                'name': name,
                'path': path,
            }
        rospy.Service('/mercury/dynparam/defaults',
                      Default, self.default_handler)
        rospy.Service('/mercury/dynparam/load', Load, self.load_handler)

    def load(self, name, path):
        if not os.path.exists(path):
            return False
        
        if not name.startswith('/'):
            name = '/' + name

        f = file(path, 'r')
        try:
            params = {}
            for doc in yaml.load_all(f.read(), Loader=yaml.FullLoader):
                params.update(doc)
        finally:
            f.close()

        try:
            self.client = DynamicReconfigureClient(name)
            try:
                self.client.update_configuration(params)
            except DynamicReconfigureCallbackException as err:
                logger.log_error(str(err))
                return False
            except DynamicReconfigureParameterException as err:
                logger.log_error(str(err))
                return False
            return True
        finally:
            self.client.close()

    def default_handler(self, request):
        key = request.name
        if key not in self.defaults:
            return False
        logger.log_warn("loading default parameters for %s ...", key)
        return self.load(self.defaults[key]['name'], self.defaults[key]['path'])

    def load_handler(self, request):
        if request.name == "":
            return False
        return self.load(request.name, request.path)

    def spin(self):
        rospy.spin()
