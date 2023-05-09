import argparse
import ipfshttpclient
import subprocess
import psutil
import json
import math
import queue
import time

from SecureShell import ShellHandler
from Simulator import sim

# System hosting HLF node related info
server_IP = "IP_HLF_node"
server_username = "username_HLF_node"
pkey_path = "path/to/private/key"


def parse_args():
	parser = argparse.ArgumentParser(description='Describe the simulation environment')

	call_args = parser.add_argument_group("Execution options")
	call_args.add_argument('-r', '--run', action='store_true', help='Runs the CARLA simulation server')
	call_args.add_argument('-s', '--simulate', action='store_true', help='Sets the simulation environment')
	call_args.add_argument('--invoke', action='store_true', help='Specifies function to invoke')
	call_args.add_argument('--query', choices=["ReadCam0Data", "ReadCam1Data", "ReadLIDARData", "ReadSpeedData",\
							"ReadAcclnData", "ReadGyroData", "ReadCompassData", "ReadThrottleData", "ReadSteerData",\
							"ReadBrakeData", "ReadGearData", "ReadHandBrakeData", "ReadVehicleFrames"],\
								help='Specifies function to invoke')
	call_args.add_argument('--user', choices=["Alice", "Bob", "Cathy"], default="Alice", help='Specifies user that queried')

	sim_args = parser.add_argument_group("Simulation parameters")
	sim_args.add_argument('--town', action='store', type=str, default='', help='Town to be stimulated')
	sim_args.add_argument('--vehicle', action='store', type=str, default='lincoln.mkz_2020', help='Name of the vehicle to be monitored')
	sim_args.add_argument('--vid', action='store', type=int, default='1', help='ID of the monitored vehicle')
	sim_args.add_argument('--frames', action='store', type=int, default='5', help='Number of frames to be monitored')
	sim_args.add_argument('--start_time', action='store', type=float, default='1', help='Start time from when data is to be retrieved')
	sim_args.add_argument('--end_time', action='store', type=float, default='11', help='End time till when data is to be retrieved')
	sim_args.add_argument('-n', '--npcs', action='store', type=int, default='30', help='Number of vehicles in the environment')
	sim_args.add_argument('-t', '--time_step', action='store', type=float, default='0.05', help='Time step for simulation')
	sim_args.add_argument('-p', '--port', action='store', type=int, default='2000', help='Port to connect client')

	return parser.parse_args()

def exec_cmd(command):
	proc = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	out, err = proc.communicate()
	return out.decode()

def run_sim(args):
	arg_list = list()
	arg_list.append("-carla-port=" + str(args.port))
	proc = subprocess.Popen(["C:\\Users\\akars\\CARLA\\CarlaUE4.exe"] + arg_list)
	time.sleep(20)
	return proc

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()


def main():
	args = parse_args()

	if args.invoke and not args.simulate:
		raise Exception("Can't push data without simulating")

	if args.run:
		proc = run_sim(args)

	if args.simulate:
		env = sim(args)
		vehicle = env.setup_env(args)
		cam0, cam1, lidar, imu = env.setup_sensors(args, vehicle)

		if args.invoke:
			frames_cam0 = [args.frames]
			frames_cam1 = [args.frames]
			frames_lid = [args.frames]
			frames_imu = [args.frames]
			data_list_cam0 = queue.Queue()
			data_list_cam1 = queue.Queue()
			data_list_lidar = queue.Queue()
			data_list_imu = queue.Queue()

			time.sleep(1)

			cam0.listen(lambda image: env.camera_callback(image, data_list_cam0, frames_cam0))
			cam1.listen(lambda image: env.camera_callback(image, data_list_cam1, frames_cam1))
			lidar.listen(lambda point_cloud: env.lidar_callback(point_cloud, data_list_lidar, frames_lid))
			imu.listen(lambda data: env.imu_callback(data, data_list_imu, frames_imu))

			time.sleep(5)

			### Uncomment to save the visual inputs as images
			# lidar.listen(lambda point_cloud: point_cloud.save_to_disk('lidar_output/%.6d.ply' % point_cloud.frame))
			# cam.listen(lambda image: image.save_to_disk('rgb_output/%06d.png' % image.frame))

			client = ipfshttpclient.connect('/ip4/10.14.75.128/tcp/5001/http')
			obj = ShellHandler(server_IP, server_username, pkey_path)

			i = 1
			while not data_list_cam0.empty():
				# Read sensor & actuator data
				cam0_data = data_list_cam0.get(block=True)
				cam1_data = data_list_cam1.get(block=True)
				lid_data = data_list_lidar.get(block=True)
				imu_data = data_list_imu.get(block=True)

				v = vehicle.get_velocity()
				speed = (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
				c = vehicle.get_control()

				# Upload to IPFS
				try:
					lst_c0 = cam0_data['image'].tolist()
					lst_c1 = cam1_data['image'].tolist()
					lst_lid = lid_data['point_cloud'].tolist()

					gyro = imu_data['gyro']
					lst_gyro = [gyro.x, gyro.y, gyro.z]

					cid_cam0 = client.add_json(lst_c0)
					cid_cam1 = client.add_json(lst_c1)
					cid_lid = client.add_json(lst_lid)
					cid_speed = client.add_json(speed)
					cid_accln = client.add_json(imu_data['accln'])
					cid_gyro = client.add_json(lst_gyro)
					cid_compass = client.add_json(imu_data['compass'])

					cid_throttle = client.add_json(c.throttle)
					cid_steer = client.add_json(c.steer)
					cid_brake = client.add_json(c.brake)
					cid_gear = client.add_json(c.gear)
					cid_hbrake = client.add_json(c.hand_brake)
					
				except Exception as error_message:
					print("Couldn't run command")
					print(error_message)

				data = {
					"FrameNo": cam0_data['frame'],

					# Sensor data
					"RGBCam0": cid_cam0,
					"RGBCam1": cid_cam1,
					"LIDAR": cid_lid,
					"Speed": cid_speed,
					"Accln": cid_accln,
					"Gyro": cid_gyro,
					"Compass": cid_compass,

					# Actuator data
					"Throttle": cid_throttle,
					"Steering": cid_steer,
					"Braking": cid_brake,
					"Gear": cid_gear,
					"HandBrake": cid_hbrake
				}
				
				str_list = ["node", "invoke.js", args.user, "PushData", str(args.vid),\
									str(cam0_data['timestamp']), json.dumps(data)]
				cmd = " ".join(str_list)
				obj.exec_cmd(cmd)

				print("Uploaded frame " + str(i))
				print()
				i = i + 1
			
			cam0.stop()
			cam1.stop()
			lidar.stop()
			imu.stop()

		env.destroy()
		if args.run:
			kill(proc.pid)
	
	if args.query != None:
		obj = ShellHandler(server_IP, server_username, pkey_path)

		if args.query == "ReadCam0Data" or args.query == "ReadCam1Data" or args.query == "ReadLIDARData"\
			or args.query == "ReadSpeedData" or args.query=="ReadAcclnData" or args.query == "ReadGyroData"\
			or args.query == "ReadCompassData" or args.query == "ReadThrottleData" or args.query == "ReadSteerData"\
			or args.query == "ReadBrakeData" or args.query == "ReadGearData" or args.query == "ReadHandBrakeData":
			
			str_list = ["node", "invoke.js", args.user, args.query, str(args.vid), str(args.start_time), str(args.end_time)]
			cmd = (" ").join(str_list)
			obj.exec_cmd(cmd, return_cid=True)

		else:
			str_list = ["node", "invoke.js", args.user, "ReadVehicleFrames", str(args.vid)]
			cmd = (" ").join(str_list)
			obj.exec_cmd(cmd, return_cid=True)


if __name__=="__main__":
    main()