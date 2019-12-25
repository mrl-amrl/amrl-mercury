import rospy
import yaml
import os

from mercury import logger
from mercury_dynparam.srv import Load
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from dynamic_reconfigure import DynamicReconfigureCallbackException


class DynamicConfigurationParameters:
    def __init__(self):
        rospy.Service('/mercury/dynparam/load', Load, self.load_handler)

    def load_handler(self, request):
        if request.name == "":            
            return False
        
        if not os.path.exists(request.path):
            return False

        f = file(request.path, 'r')
        try:
            params = {}
            for doc in yaml.load_all(f.read(), Loader=yaml.FullLoader):
                params.update(doc)
        finally:
            f.close()

        try:
            self.client = DynamicReconfigureClient(request.name)        
            try:
                self.client.update_configuration(doc)
            except DynamicReconfigureCallbackException as err:
                logger.log_error(str(err))
                return False
            return True
        finally:
            self.client.close()

    def spin(self):
        rospy.spin()
