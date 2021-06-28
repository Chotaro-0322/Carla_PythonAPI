#!/usr/bin/env python
# -*- coding: utf-8 -*-
import carla
from carla import Transform, Location, Rotation
from agents.navigation.controller import VehiclePIDController

import random
import time
import math
from multiprocessing import Process
import threading
#import pcl
# import pdb


class PedestrianGenerator():
    def __init__(self, worls, map, blueprint_library, number_pedestrian):
        self.number_vehicle = number_pedestrian
        self.blueprint_vehicle = blueprint_library
        self.world = self.world
        self.map = mapself.vehicle_status = {}

    def create_pedestrian(self):
        self.waker_bp = self.world.get_blueprint_library().filter("walker.pedestrian.")
        self.controller_bp = self.world.get_blueprint_library().find("controller.ai.walker")

class VehicleGenerator():
    def __init__(self, world, map, blueprint_library, number_vehicle):
        self.number_vehicle = number_vehicle
        self.blueprint_library = blueprint_library
        self.world = world
        self.map = map
        self.vehicle_status = {}

    def create_vehicle(self):
        position_list = []
        self.vehicle_actor_list = []
        self.rad_actor_list = []
        self.controller_list = []
        for n in range(self.number_vehicle):
            if n == 0:
                vehicle_transform = carla.Transform(carla.Location(x=50, y=0, z=0), carla.Rotation(pitch=0, yaw=0, roll=0))
                vehicle_transform.rotation.yaw += 180
                position_list.append(vehicle_transform)
            if n == 1:
                vehicle_transform = carla.Transform(carla.Location(x=-50, y=0, z=0), carla.Rotation(pitch=0, yaw=0, roll=0))
                vehicle_transform.rotation.yaw += 180
                position_list.append(vehicle_transform)
        num_lost_vehicle = 0
        for n, position in enumerate(position_list):
            try:
                self.vehicle_bp = random.choice(self.blueprint_library.filter("vehicle.**.*"))
                self.vehicle_actor = self.world.spawn_actor(self.vehicle_bp, position)
                self.custom_controller = VehiclePIDController(self.vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
                self.vehicle_actor_list.append(self.vehicle_actor)
                self.controller_list.append(self.custom_controller)

                # 最初は全て衝突していない
                self.vehicle_status[str(n - num_lost_vehicle)] = False

            except RuntimeError:
                print("Warnig : 車の出現位置が被りました")
                num_lost_vehicle += 1
                pass
            #self.vehicle_actor.set_autopilot(True)
        # print("車の状態は", self.vehicle_status)


        if self.number_vehicle == 1:
            return self.world
        else:
            return self.world

    def vehicle_transport(self):
        # print("vehicle num is ", vehicle_num, "collision is ", collision)
        # print("vehice status is ", self.vehicle_status)
        # print("check !!!!!")
        for i, vehicle in enumerate(self.vehicle_actor_list):
            if self.vehicle_status[str(i)] == False:
                waypoint = self.map.get_waypoint(vehicle.get_location())
                waypoint = random.choice(waypoint.next(5))
                controll_signal = self.controller_list[i].run_step(20, waypoint)
                self.vehicle_actor_list[i].apply_control(controll_signal)
            else:
                controll_signal = carla.VehicleControl(brake=1.0)
                self.vehicle_actor_list[i].apply_control(controll_signal)

class World():
    def __init__(self):
        # クライアントとマップの取得

        '''
        simultor need to set "Client". and this need to provide the IP and port
        '''
        client = carla.Client("localhost", 2000)

        '''
        somethime, time-out is needed.
        '''
        client.set_timeout(10.0)

        '''
        If we have the client, we can directly retrieve the self.world.
        '''
        #print(client.get_available_maps())
        self.world = client.load_world("/Game/Carla/Maps/siskou_only_road")

        # 設計図
        # LiDARのチャンネルなどの設定
        '''
        blueprint_library内に含まれる設計図のリストを作成
        '''
        blueprint_library = self.world.get_blueprint_library()

        # 観客視点とマップとwayPointの設定
        self.spectator = self.world.get_spectator()
        # self.spectator.set_transform(carla.Transform(carla.Location(x=90, y=131, z=10), carla.Rotation(pitch=-40, yaw=95)))
        self.spectator.set_transform(carla.Transform(carla.Location(x=0.0, y=0, z=10), carla.Rotation(pitch=-40, yaw=95)))
        self.map = self.world.get_map()

        waypoint_list = self.map.generate_waypoints(2.0)
        waypoint_tuple_list = self.map.get_topology()
        self.visualize_waypoint(waypoint_tuple_list)

        # スポーンポイントを表示(test)
        spawnpoint_list = self.world.get_map().get_spawn_points()
        self.visualize_location(spawnpoint_list)

        '''
        IDによる設計図の選択.
        '''
        # Chose a vehicle blueprint
        self.GestVehicle = VehicleGenerator(self.world, self.map, blueprint_library, 2)
        # self.MainVehicle = VehicleGenerator(self.world, self.map, blueprint_library, 1)

        # self.world = self.MainVehicle.create_vehicle()
        self.world = self.GestVehicle.create_vehicle()

        self.camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '1920')
        self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('fov', '200')

        # アクターの設置場所の決定
        camera_transform = carla.Transform(carla.Location(x=0, y=0, z=1.0), carla.Rotation(pitch=0, yaw=95))
        self.camera_actor = self.world.spawn_actor(self.camera_bp, camera_transform)

        # 天気の変更
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=30.0,
            sun_altitude_angle=70.0)

        self.world.set_weather(weather)

        print(self.world.get_weather())


    def visualize_waypoint(self, waypoint):
        for cuple_point in waypoint:
            # print("point is ", cuple_point[0])
            self.world.debug.draw_line(
                cuple_point[0].transform.location + carla.Location(z=0),
                cuple_point[1].transform.location + carla.Location(z=0),
                thickness=0.025,
                color=carla.Color(0, 255, 255))

    def visualize_location(self, point_list):
        # print("point is ", point_list)
        for point in point_list:
            # print("point is ", point)
            self.world.debug.draw_point(point.location + carla.Location(z=0.1),
                                            color=carla.Color(r=0, g=255, b=0), life_time = -1.0,
                                        )

    def update(self):
        while True:
            self.GestVehicle.vehicle_transport()
            time.sleep(0.001)

if __name__ == "__main__":
    World = World()
    World.update()
