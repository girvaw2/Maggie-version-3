import roslib
roslib.load_manifest('pi_head_tracking_3d_part1')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed
from dynamixel_driver import dynamixel_io

class Relax():
  def __init__(self):
    self.port_name = '/dev/ttyUSB0'
    self.baud_rate = 1000000
    
  def connect(self):
    try:
      self.dxl_io = dynamixel_io.DynamixelIO(self.port_name, self.baud_rate)
    except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)
  
  def set_id(self, old_id, new_id):
    try:
      self.dxl_io.set_id(old_id, new_id)
    except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)  
  
if __name__=='__main__':
  set_id = Relax()
  set_id.connect()
  
  ' set the old and new servo IDs:
  '
  set_id.set_id(1, 2)
