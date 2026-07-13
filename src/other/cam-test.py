import pyrealsense2 as rs

try:
	context = rs.context()
	devices = context.query_devices()
	
	if len(devices) == 0:
		print("RealSense SDK imported fine, but NO camera was detected. Check USB connection")
	else:
		print(f"Success! Detected device: {devices[0].get_info(rs.camera_info.name)}")
	
except Execution as e:
	print(f"Error communicating with the camera: {e}")
