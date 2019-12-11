import rospy
from mercury_common.srv import SetEnabled, SetEnabledRequest

class PowerController:
    def __init__(self):
        pass
    
    def send(self, name, state):
        proxy = rospy.ServiceProxy('/mercury/power/' + name, SetEnabled)    
        request = SetEnabledRequest()
        request.enabled = state            
        response = proxy.call(request)
        return response.state