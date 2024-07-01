import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from PIL import Image as PIL_Image

class Follower:
  def __init__(self):
        rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.image_transformation_cb)
        self.pub_cmd_vel = rospy.Publisher("/r2d2_diff_drive_controller/cmd_vel", Twist, queue_size=1, latch=False)
        self.velocity = Twist()
        self.frame = None

  def image_transformation_cb(self, msg):
    im_h = msg.height
    im_w = msg.width

    img = PIL_Image.frombuffer('RGB', (im_h, im_w), msg.data)
    self.frame = np.array(img)

  def get_track_line_center(self, image, mask):
    h, w, d = image.shape
    search_top = 3*h//4
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      cv2.circle(image, (w//2,cy-60), 10, (255, 0,0), -1)
      return cx, cy
    else: 
      return w//2, h//2

  def get_color_mask(self, hsv):
      lower_yellow = np.array([ 50,  50, 170])
      upper_yellow = np.array([255, 255, 190])
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
      return mask

  def calculate_control_output(self,cx, cy):
    h, w, d = self.frame.shape
    error = w/2 - cx
    return error * 0.005


  def loop(self):
      # infinite loop
      while True:
          if self.frame is None:
              continue

          frame = self.frame.copy()
            
          frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
          frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

          mask = self.get_color_mask(frame_hsv)
          cx, cy = self.get_track_line_center(frame_rgb, mask)
          angular_vel = self.calculate_control_output(cx, cy)

          self.velocity.linear.x = 0.1
          self.velocity.angular.z = angular_vel
          self.pub_cmd_vel.publish(self.velocity)

          # Show output window
          cv2.imshow("Output Frame", frame_rgb)

          key = cv2.waitKey(1) & 0xFF
          
          # check for 'q' key-press
          if key == ord("q"):
              #if 'q' key-pressed break out
              break

      cv2.destroyAllWindows()   


if __name__ == '__main__': 
  rospy.init_node('follower')
  follower = Follower()
  follower.loop()
