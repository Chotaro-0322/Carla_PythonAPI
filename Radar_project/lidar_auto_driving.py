#!/usr/bin/env python
# -*- coding: utf-8 -*-
import carla
from carla import Transform, Location, Rotation
from agents.navigation.controller import VehiclePIDController
import numpy as np
from mayavi import mlab

import random
import time
import math
import threading

def mat_init():
    pass

class VehicleGenerator():
    def __init__(self, world, vehicle_bp, number_vehicle):
        self.number_vehicle = number_vehicle
        self.vehicle_bp = vehicle_bp
        self.world = world

    def create_vehicle(self):
        position_list = []
        vehicle_actor_list = []
        controll_list = []
        for n in range(self.number_vehicle):
            #spawn_point = carla.Transform()
            #spawn_point.location = self.world.get_random_location_from_navigation()
            vehicle_transform = random.choice(self.world.get_map().get_spawn_points())
            # print("transform is ", vehicle_transform)
            position_list.append(vehicle_transform)

        for n, position in enumerate(position_list):
            print("position is ", position)
            self.vehicle_actor = self.world.spawn_actor(self.vehicle_bp, position)
            self.custom_controller = VehiclePIDController(self.vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

            # self.vehicle_actor.set_autopilot(True)

            #location = self.vehicle_actor.get_location()
            #location.z += 1
            #self.vehicle_actor.set_location(location)

            vehicle_actor_list.append(self.vehicle_actor)
            controll_list.append(self.custom_controller)

        if self.number_vehicle == 1:
            return self.world, vehicle_actor_list[0], controll_list[0]
        else:
            return self.world, vehicle_actor_list, controll_list



class World():
    def __init__(self):
        # クライアントとマップの取得

        """
        simultor need to set "Client". and this need to provide the IP and port
        """
        client = carla.Client("localhost", 2000)

        """
        somethime, time-out is needed.
        """
        client.set_timeout(10.0)

        """
        If we have the client, we can directly retrieve the self.world.
        """
        #print(client.get_available_maps())
        self.world = client.load_world("/Game/Carla/Maps/Town04")

        # 設計図
        # LiDARのチャンネルなどの設定
        """
        blueprint_library内に含まれる設計図のリストを作成
        """
        blueprint_library = self.world.get_blueprint_library()

        """
        IDによる設計図の選択.
        """
        # Chose a vehicle blueprint
        vehicle_bp = random.choice(blueprint_library.filter("vehicle.**.*"))
        # self.world, self.gest_vehicle_act_list, self.gest_controll_list = VehicleGenerator(self.world, vehicle_bp, 1).create_vehicle()
        self.world, self.vehicle_actor, self.controll_list = VehicleGenerator(self.world, vehicle_bp, 1).create_vehicle()

        self.camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        self.camera_bp.set_attribute("image_size_x", "1920")
        self.camera_bp.set_attribute("image_size_y", "1080")
        self.camera_bp.set_attribute("fov", "110")

        self.lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
        self.lidar_bp.set_attribute("channels", str(16))
        self.lidar_bp.set_attribute("range", str(100))
        self.lidar_bp.set_attribute('noise_stddev', '0.2')
        # self.lidar_bp.set_attribute("horizontal_fov", str(360))
        # self.lidar_bp.set_attribute("upper_fov", str(15))
        # self.lidar_bp.set_attribute("lower_fov", str(-15))
        self.lidar_bp.set_attribute("rotation_frequency", str(1/0.05))

        lid_location = carla.Location(x=-0.5, z=2.0)
        lid_rotation = carla.Rotation(pitch=0)

        # アクターの設置場所の決定
        camera_transform = carla.Transform(carla.Location(x=-20, z=3), carla.Rotation(pitch=-10))
        self.camera_actor = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.vehicle_actor)

        self.lid_transform = carla.Transform(lid_location,lid_rotation)
        self.lid_ego = self.world.spawn_actor(self.lidar_bp,self.lid_transform,attach_to=self.vehicle_actor, attachment_type=carla.AttachmentType.Rigid)

        # 天気の変更
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=30.0,
            sun_altitude_angle=70.0)

        self.world.set_weather(weather)

        print(self.world.get_weather())

        # マップとwayPointの設定
        self.spectator = self.world.get_spectator()
        self.spectator.set_transform(carla.Transform(carla.Location(z=50), carla.Rotation(pitch=-90)))
        self.map = self.world.get_map()

        waypoint_list = self.map.generate_waypoints(2.0)
        waypoint_tuple_list = self.map.get_topology()
        self.visualize_waypoint(waypoint_tuple_list)


    def visualize_waypoint(self, waypoint):
        for cuple_point in waypoint:
            #print("point is ", cuple_point[0])
            self.world.debug.draw_line(
                cuple_point[0].transform.location + carla.Location(z=0),
                cuple_point[1].transform.location + carla.Location(z=0),
                thickness=0.025,
                color=carla.Color(0, 255, 255))


    def lid_callback(self, lidar_data):
        #print("lidar_data is ", lidar_data.raw_data)
        data = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype("f4")))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))
        # print("data is ", data)

        intensity = data[:, -1]
        points = data[:, :-1]
        points[:, :1] = -points[:, :1]

        self.buf["pts"] = points
        self.buf["intensity"] = intensity

    def carlaEventLoop(self, world):
        while True:
            self.spectator.set_transform(self.camera_actor.get_transform())
            # for vehicle, controll in zip(self.gest_vehicle_act_list, self.gest_controll_list):
            #     waypoint = self.map.get_waypoint(vehicle.get_location())
            #     waypoint = random.choice(waypoint.next(5))
            #     controll_signal = controll.run_step(30, waypoint)
            #     vehicle.apply_control(controll_signal)

            waypoint = self.map.get_waypoint(self.vehicle_actor.get_location())
            waypoint = random.choice(waypoint.next(1))
            controll_signal = self.controll_list.run_step(30, waypoint)
            self.vehicle_actor.apply_control(controll_signal)

            time.sleep(0.05)
    def update(self):
        self.buf = {"pts": np.zeros((1,3)), "intensity":np.zeros(1)}
        self.lid_ego.listen(lambda lidar_data: self.lid_callback(lidar_data))

        worldThread = threading.Thread(target=self.carlaEventLoop, args=[self.world], daemon=True)
        worldThread.start()

        vis = mlab.points3d([6, 2, 3, 4], [1, 2, 3, 2], [1, 1, 3, 5], mode="point", figure=mlab.figure(bgcolor=(0, 0, 0)))
        @mlab.animate(delay=10)
        def updateAnimation():
            while True:
                vis.mlab_source.reset(x=self.buf["pts"][:, 0], y=self.buf["pts"][:, 1], z=self.buf["pts"][:, 2], color=(0, 1, 1))
                yield

        updateAnimation()
        mlab.show()


# if __name__ == "__main__":
World = World()

World.update()
# mlab.show()

