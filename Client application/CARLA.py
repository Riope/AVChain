import argparse
import carla
import ipfshttpclient
import subprocess
import psutil
import paramiko
import json
import math
import re
import queue
import numpy as np
import random
import time

# import openpyxl
# file_path = "../results/existing_file.xlsx"

client_IP = 'localhost'
server_IP = "10.14.75.128"
server_username = "akarsh"
pkey_path = "/home/ssural/.ssh/id_rsa"

class sim:
	def __init__(self, args) -> None:
		self.client = carla.Client(client_IP, args.port)
		self.client.set_timeout(10.0)

		if args.town == '':
			self.world = self.client.get_world()
		else:
			self.world = self.client.load_world('Town'+args.town)
		self.bp_lib = self.world.get_blueprint_library()
		self.spawn_pts = self.world.get_map().get_spawn_points()
		self.actorList = list()

		settings = self.world.get_settings()
		settings.fixed_delta_seconds = args.time_step
		self.world.apply_settings(settings)

	def setup_env(self, args):
		vehicle_bp = self.bp_lib.find('vehicle.' + args.vehicle)
		vehicle = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_pts))
		self.actorList.append(vehicle)

		for i in range(args.npcs):
			vehicle_bp = random.choice(self.bp_lib.filter('vehicle'))
			npc = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_pts))
			if npc is not None:
				self.actorList.append(npc)

		for v in self.world.get_actors().filter('*vehicle*'):
			v.set_autopilot(True)

		return vehicle

	def setup_sensors(self, args, vehicle):
		#0.1m added to height compared to KITTI to adjust for shape of vehicle and remove occlusion
		cam_bp_0 = self.bp_lib.find('sensor.camera.rgb')
		cam_init_0 = carla.Transform(carla.Location(x=0.27, y=0, z=1.75))
		cam_bp_0.set_attribute('image_size_x', str(1392))
		cam_bp_0.set_attribute('image_size_y', str(512))
		cam_bp_0.set_attribute('fov', str(72))
		cam_bp_0.set_attribute('sensor_tick', str(args.time_step))
		cam_0 = self.world.spawn_actor(cam_bp_0, cam_init_0, attach_to=vehicle)
		self.actorList.append(cam_0)

		cam_bp_1 = self.bp_lib.find('sensor.camera.rgb')
		cam_init_1 = carla.Transform(carla.Location(x=0.27, y=0.54, z=1.75))
		cam_bp_1.set_attribute('image_size_x', str(1392))
		cam_bp_1.set_attribute('image_size_y', str(512))
		cam_bp_1.set_attribute('fov', str(72))
		cam_bp_1.set_attribute('sensor_tick', str(args.time_step))
		cam_1 = self.world.spawn_actor(cam_bp_1, cam_init_1, attach_to=vehicle)
		self.actorList.append(cam_1)

		lidar_bp = self.bp_lib.find('sensor.lidar.ray_cast')
		lidar_bp.set_attribute('channels',str(64))
		lidar_bp.set_attribute('points_per_second',str(1300000)) 
		lidar_bp.set_attribute('rotation_frequency',str(10))
		lidar_bp.set_attribute('range',str(120))
		lidar_bp.set_attribute('upper_fov',str(2.0))
		lidar_bp.set_attribute('lower_fov',str(-24.8))
		lidar_location = carla.Location(0,0,2.5)
		lidar_rotation = carla.Rotation(0,0,0)
		lidar_transform = carla.Transform(lidar_location,lidar_rotation)
		lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
		self.actorList.append(lidar)

		imu_bp = self.bp_lib.find('sensor.other.imu')
		imu_bp.set_attribute('sensor_tick', str(args.time_step))
		imu_bp.set_attribute('noise_accel_stddev_x', '0.01')
		imu_bp.set_attribute('noise_accel_stddev_y', '0.01')
		imu_bp.set_attribute('noise_accel_stddev_z', '0.01')
		imu_bp.set_attribute('noise_gyro_stddev_x', '0.01')
		imu_bp.set_attribute('noise_gyro_stddev_y', '0.01')
		imu_bp.set_attribute('noise_gyro_stddev_z', '0.01')
		imu_transform = carla.Transform(carla.Location(x=1.0, y=0.0, z=1.0))
		imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
		self.actorList.append(imu)

		return cam_0, cam_1, lidar, imu
	
	def destroy(self):
		self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actorList])

class ShellHandler:
	def __init__(self, host, user, path):
		self.ssh = paramiko.SSHClient()
		self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
		self.ssh.connect(host, username=user, key_filename=path, port=22)
		# self.ssh.connect(host, username=user, password=path, port=22)

		channel = self.ssh.invoke_shell()
		self.stdin = channel.makefile('wb')
		self.stdout = channel.makefile('r')

	def __del__(self):
		self.ssh.close()

	def execute(self, cmd):
		cmd = cmd.strip('\n')
		self.stdin.write(cmd + '\n')
		finish = 'end of stdOUT buffer. finished with exit status'
		echo_cmd = 'echo {} $?'.format(finish)
		self.stdin.write(echo_cmd + '\n')
		shin = self.stdin
		self.stdin.flush()

		shout = []
		sherr = []
		exit_status = 0
		for line in self.stdout:
			if str(line).startswith(cmd) or str(line).startswith(echo_cmd):
				# up for now filled with shell junk from stdin
				shout = []
			elif str(line).startswith(finish):
				# our finish command ends with the exit status
				exit_status = int(str(line).rsplit(maxsplit=1)[1])
				if exit_status:
					sherr = shout
					shout = []
				break
			else:
				# get rid of 'coloring and formatting' special characters
				shout.append(re.compile(r'(\x9B|\x1B\[)[0-?]*[ -/]*[@-~]').sub('', line).
								replace('\b', '').replace('\r', '').replace('\n',''))

		# first and last lines of shout/sherr contain a prompt
		if shout and echo_cmd in shout[-1]:
			shout.pop()
		if shout and cmd in shout[0]:
			shout.pop(0)
		if sherr and echo_cmd in sherr[-1]:
			sherr.pop()
		if sherr and cmd in sherr[0]:
			sherr.pop(0)

		return shin, shout, sherr

	def exec_cmd(self, cmd, return_cid=False):
		try:
			_, std_out, _ = self.execute(cmd)
			output = {"std_out": std_out}

			if return_cid:
				temp_str = std_out[-2]
				temp_str = temp_str[temp_str.index('[')+1:temp_str.index(']')]
				output['cids'] = temp_str.split(",")

			print(json.dumps(output))
			
		except Exception as error_message:
			print("Couldn't run command")
			print(error_message)

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
	try:
		pid = subprocess.check_output(["lsof", "-t", "-i:2000"]).strip()
		if pid:
			subprocess.call(["kill", pid])
	except:
		pass

	arg_list = list()
	arg_list.append("-RenderOffScreen")
	arg_list.append("-carla-port=" + str(args.port))
	proc = subprocess.Popen(["/home/ssural/CARLA/CarlaUE4.sh"] + arg_list)
	time.sleep(20)
	return proc

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()

def camera_callback(image, data_list, frames):
	if frames[0] <= 0:
		return
	
	# if frames[0]%20 == 0:
	output = {'frame': 0, 'timestamp': 0.0, 'image': np.zeros((image.height, image.width, 4))}
	output['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
	output['frame'] = image.frame
	output['timestamp'] = image.timestamp

	data_list.put(output, block=True)

	# time_taken = "{:.8f}".format(time.time() - start_time)
	# sheet.cell(row=1+frames[0], column=2+cam_no, value=time_taken)

	frames[0] = frames[0] -1

def lidar_callback(point_cloud, data_list, frames):
	if frames[0] <= 0:
		return

	# if frames[0]%20 == 0:
	pts = int(point_cloud.raw_data.shape[0] / 4)
	output = {'frame': 0, 'timestamp': 0.0, 'point_cloud': np.zeros((pts, 4))}
	# output['point_cloud'] = np.reshape(np.copy(point_cloud.raw_data), (pts, 4))

	output['point_cloud'] = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
	output['point_cloud'] = np.reshape(output['point_cloud'], (int(output['point_cloud'].shape[0] / 4), 4))

	output['frame'] = point_cloud.frame
	output['timestamp'] = point_cloud.timestamp

	data_list.put(output, block=True)

	# time_taken = "{:.8f}".format(time.time() - start_time)
	# sheet.cell(row=1+frames[0], column=4, value=time_taken)

	frames[0] = frames[0] -1

def imu_callback(data, data_list, frames):
	if frames[0] <= 0:
		return

	# if frames[0]%20 == 0:
	output = {'frame': 0, 'timestamp': 0.0, 'accln': 0.0, 'gyro': carla.Vector3D(), 'compass': 0.0}

	accln = data.accelerometer
	output['accln'] = math.sqrt(accln.x**2 + accln.y**2 + accln.z**2)
	output['gyro'] = data.gyroscope
	output['compass'] = data.compass

	output['frame'] = data.frame
	output['timestamp'] = data.timestamp

	data_list.put(output, block=True)

	# time_taken = "{:.8f}".format(time.time() - start_time)
	# sheet.cell(row=1+frames[0], column=4, value=time_taken)

	frames[0] = frames[0] -1

def get_data_size(ipfs_client, file_hash):
    block_contents = ipfs_client.cat(file_hash)
    return len(block_contents)

def val(query):
	if query == "ReadCam0Data":
		return 0
	if query == "ReadCam1Data":
		return 1
	if query == "ReadLIDARData":
		return 2
	if query == "ReadSpeedData":
		return 3
	if query == "ReadThrottleData":
		return 4
	if query == "ReadSteerData":
		return 5
	if query == "ReadBrakeData":
		return 6
	if query == "ReadGearData":
		return 7
	if query == "ReadHandBrakeData":
		return 8
	return -99

def main():
	args = parse_args()

	if args.invoke and not args.simulate:
		raise Exception("Can't push data without simulating")

	if args.run:
		proc = run_sim(args)

	# wb = openpyxl.load_workbook(file_path)
	# sheet = wb.active

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

			# sheet.cell(row=1, column=1, value="frame no")
			# start_time = time.time()

			cam0.listen(lambda image: camera_callback(image, data_list_cam0, frames_cam0))
			cam1.listen(lambda image: camera_callback(image, data_list_cam1, frames_cam1))
			lidar.listen(lambda point_cloud: lidar_callback(point_cloud, data_list_lidar, frames_lid))
			imu.listen(lambda data: imu_callback(data, data_list_imu, frames_imu))

			time.sleep(5)
			# lidar.listen(lambda point_cloud: point_cloud.save_to_disk('lidar_output/%.6d.ply' % point_cloud.frame))
			# cam.listen(lambda image: image.save_to_disk('out/%06d.png' % image.frame))

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

				# st_time = time.time()
				obj.exec_cmd(cmd)
				# time_taken = "{:.8f}".format(time.time() - st_time)
				# sheet.cell(row=1+i, column=2, value=time_taken)

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

			# sheet.cell(row=1, column=2+val(args.query), value=args.query)
			# st_time = time.time()
			obj.exec_cmd(cmd, return_cid=True)
			# time_taken = "{:.8f}".format(time.time() - st_time)
			# sheet.cell(row=5, column=2+val(args.query), value=time_taken)

		else:
			str_list = ["node", "invoke.js", args.user, "ReadVehicleFrames", str(args.vid)]
			cmd = (" ").join(str_list)

			# sheet.cell(row=1, column=11, value="ReadVehicleFrames")
			# st_time = time.time()
			obj.exec_cmd(cmd, return_cid=True)
			# time_taken = "{:.8f}".format(time.time() - st_time)
			# sheet.cell(row=3, column=11, value=time_taken)

	# wb.save(file_path)

if __name__=="__main__":
    main()
