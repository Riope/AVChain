import carla
import math
import numpy as np
import random

# CARLA server related info
client_IP = 'localhost'


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
        vehicle = self.world.try_spawn_actor(
            vehicle_bp, random.choice(self.spawn_pts))
        self.actorList.append(vehicle)

        for i in range(args.npcs):
            vehicle_bp = random.choice(self.bp_lib.filter('vehicle'))
            npc = self.world.try_spawn_actor(
                vehicle_bp, random.choice(self.spawn_pts))
            if npc is not None:
                self.actorList.append(npc)

        for v in self.world.get_actors().filter('*vehicle*'):
            v.set_autopilot(True)

        return vehicle

    def setup_sensors(self, args, vehicle):
        # 0.1m added to height compared to KITTI to adjust for shape of vehicle and remove occlusion
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
        lidar_bp.set_attribute('channels', str(64))
        lidar_bp.set_attribute('points_per_second', str(1300000))
        lidar_bp.set_attribute('rotation_frequency', str(10))
        lidar_bp.set_attribute('range', str(120))
        lidar_bp.set_attribute('upper_fov', str(2.0))
        lidar_bp.set_attribute('lower_fov', str(-24.8))
        lidar_location = carla.Location(0, 0, 2.5)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar = self.world.spawn_actor(
            lidar_bp, lidar_transform, attach_to=vehicle)
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

    def camera_callback(self, image, data_list, frames):
        if frames[0] <= 0:
            return

        # if frames[0]%20 == 0:
        output = {'frame': 0, 'timestamp': 0.0,
                  'image': np.zeros((image.height, image.width, 4))}
        
        output['image'] = np.reshape(
            np.copy(image.raw_data), (image.height, image.width, 4))
        
        output['frame'] = image.frame
        output['timestamp'] = image.timestamp

        data_list.put(output, block=True)

        frames[0] = frames[0] - 1

    def lidar_callback(self, point_cloud, data_list, frames):
        if frames[0] <= 0:
            return

        # if frames[0]%20 == 0:
        pts = int(point_cloud.raw_data.shape[0] / 4)
        output = {'frame': 0, 'timestamp': 0.0,
                  'point_cloud': np.zeros((pts, 4))}

        output['point_cloud'] = np.copy(np.frombuffer(
            point_cloud.raw_data, dtype=np.dtype('f4')))
        output['point_cloud'] = np.reshape(
            output['point_cloud'], (int(output['point_cloud'].shape[0] / 4), 4))

        output['frame'] = point_cloud.frame
        output['timestamp'] = point_cloud.timestamp

        data_list.put(output, block=True)

        frames[0] = frames[0] - 1

    def imu_callback(self, data, data_list, frames):
        if frames[0] <= 0:
            return

        # if frames[0]%20 == 0:
        output = {'frame': 0, 'timestamp': 0.0, 'accln': 0.0,
                  'gyro': carla.Vector3D(), 'compass': 0.0}

        accln = data.accelerometer
        output['accln'] = math.sqrt(accln.x**2 + accln.y**2 + accln.z**2)
        output['gyro'] = data.gyroscope
        output['compass'] = data.compass

        output['frame'] = data.frame
        output['timestamp'] = data.timestamp

        data_list.put(output, block=True)

        frames[0] = frames[0] - 1

    def destroy(self):
        self.client.apply_batch([carla.command.DestroyActor(x)
                                for x in self.actorList])
