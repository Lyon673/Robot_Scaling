import message_filters
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from ambf_msgs.msg import RigidBodyState
from ambf_msgs.msg import ActuatorState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geomagic_control.msg import DeviceButtonEvent
from os.path import join
import atexit
import time
import sys
import os

stereo_l_img = []
stereo_r_img = []
segment_l_img = []
segment_r_img = []
psm_ghost_pose = []
psm_pose_list = []
psm_twist_list = []
clutch_times_list = [0]
camera_depthdata_list = []

start_time = time.time()

def kin_img_cb(psm1,psm_pose,psm_twist,camera_depthdata):
	"""
	ros approximate time synchronizer callback, invoked when target messages are approximately synced
	"""
	RigidBodyState
	sys.stdout.write('\r-- Time past: %02.1f' % float(time.time() - start_time))
	sys.stdout.flush()
	#stereo_l_img.append(bridge.imgmsg_to_cv2(stereo_l,"bgr8"))
	#stereo_r_img.append(bridge.imgmsg_to_cv2(stereo_r,"bgr8"))
	#segment_l_img.append(bridge.imgmsg_to_cv2(segment_l,"bgr8"))
	#segment_r_img.append(bridge.imgmsg_to_cv2(segment_r,"bgr8"))
	psm_ghost_pose.append(np.array([psm1.pose]+[psm1.header.stamp.secs])) # timestamp secs=sec nsecs=nsec
	psm_pose_list.append(np.array([psm_pose.pose]))
	psm_twist_list.append(np.array([psm_twist.linear]))
	camera_depthdata_list.append(np.array([camera_depthdata.data]))
	
def cal_clutch_times(button):
	if button.grey_button == 1:
		clutch_times_list[0] += 1


def save_data_cb():
	"""
	data storage callback, invoked when script terminates
	"""
	if not os.path.exists('./data'):
		os.mkdir('./data')
	print("saving data...")
	#np.save(join('./data', 'stereo-l.npy'), stereo_l_img)
	#np.save(join('./data', 'stereo-r.npy'), stereo_r_img)
	#np.save(join('./data', 'segment-l.npy'), segment_l_img)
	#np.save(join('./data', 'segment-r.npy'), segment_r_img)
	np.save(join('./data', 'psm_ghost_pose.npy'), psm_ghost_pose)
	np.save(join('./data', 'psm_pose.npy'), psm_pose_list)
	np.save(join('./data', 'psm_twist.npy'), psm_twist_list)
	np.save(join('./data', 'clutch_times.npy'), clutch_times_list)
	print("done saving...")


if __name__ == '__main__':
	
	bridge = CvBridge()

	print("started collecting PSM 1&2 baselink, stereo image, and segmentation image ...")
	print("Press Ctrl-C to stop collecting...")
	
	# init ros node
	rospy.init_node('collect_ambf_data', anonymous=True)
	
	# create subscriber
	PSM_GHOST = message_filters.Subscriber("ambf/env/ghosts/psm1/Actuator0/State", ActuatorState)
	TOOL = message_filters.Subscriber("/ambf/env/psm2/toolyawlink/State", RigidBodyState)
	PSM_POSE = message_filters.Subscriber("/Geomagic/pose", PoseStamped)
	PSM_TWIST = message_filters.Subscriber("/Geomagic/twist", Twist)
	CAMERA_DEPTHDATA = message_filters.Subscriber("/ambf/env/cameras/cameraL/DepthData", PointCloud2)
	rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, cal_clutch_times)
	
	#STEREO_L = message_filters.Subscriber("/ambf/env/cameras/cameraL/ImageData", Image)
	#STEREO_R = message_filters.Subscriber("/ambf/env/cameras/cameraR/ImageData", Image)
	#SEGMENT_L = message_filters.Subscriber("/ambf/env/cameras/segmentation_cameraL/ImageData", Image)
	#SEGMENT_R = message_filters.Subscriber("/ambf/env/cameras/segmentation_cameraR/ImageData", Image)
	
	# create approximate time synchronizer
	#ts = message_filters.ApproximateTimeSynchronizer([PSM_1, PSM_2, STEREO_L, STEREO_R, SEGMENT_L, SEGMENT_R], queue_size=10, slop=0.2, allow_headerless=False)
	ts = message_filters.ApproximateTimeSynchronizer([PSM_GHOST,PSM_POSE,PSM_TWIST], queue_size=10, slop=0.2, allow_headerless=True)
	ts.registerCallback(kin_img_cb)
	
	# spin ros until shutdown, i.e., Ctrl-C
	rospy.spin()

	# print msg container info
	#print(len(stereo_l_img))
	#print(len(stereo_r_img))
	#print(len(segment_l_img))
	#print(len(segment_r_img))
	print(len(psm_ghost_pose))
	print(len(psm_pose_list))
	print(len(psm_twist_list))

	# save data
	atexit.register(save_data_cb)