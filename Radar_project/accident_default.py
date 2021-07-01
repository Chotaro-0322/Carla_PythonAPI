#!/usr/bin/env python
# -*- coding: utf-8 -*-
import carla
from carla import Transform, Location, Rotation
from agents.navigation.controller import VehiclePIDController

import numpy as np
import random
import time
import math
from multiprocessing import Process
import threading

import pygame
from pygame.locals import *

import sys
#import pcl
# import pdb


class WalkerGenerator():
    def __init__(self, world, map, blueprint_library):
        self.blueprint_vehicle = blueprint_library
        self.world = world

    def create_walker(self):
        self.walker_bp = random.choice(self.world.get_blueprint_library().filter("walker.pedestrian.*"))
        self.controller_bp = self.world.get_blueprint_library().find("controller.ai.walker")

        self.walker_transform = carla.Transform(carla.Location(x = -20, y = -3, z = 1), carla.Rotation(pitch=0, yaw=0, roll=0))
        self.walker = self.world.spawn_actor(self.walker_bp, self.walker_transform)
        self.world.wait_for_tick()

        self.controller = self.world.spawn_actor(self.controller_bp, carla.Transform(), self.walker)

        self.controller.start()
        speed = 1.6
        # walker_location = self.world.get_random_location_from_navigation()
        walker_location = carla.Location(x = 20, y = -3, z = 1)
        self.visualize_location(walker_location)
        self.controller.go_to_location(walker_location)
        self.controller.set_max_speed(speed)

        return self.world

    def direction_set(self, direction):
        # direction = carla.WalkerControl(direction)
        self.controller.go_to_location(direction)

    def stop(self):
        self.controller.stop()

    def start(self):
        self.controller.start()

    def speed(self, speed):
        self.controller.set_max_speed(speed)


    def visualize_location(self, point):
        # print("point is ", point_list)
        # print("point is ", point)
        self.world.debug.draw_point(point + carla.Location(z=0.1),
                                        color=carla.Color(r=255, g=0, b=0), life_time = -1.0,
                                    )

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
        # vehicle light setting
        # light_state = carla.VehicleLightState.LowBeam | carla.VehicleLightState.Position
        light_state = carla.VehicleLightState(carla.VehicleLightState.LowBeam)
        # ------------------------------------------
        for n in range(self.number_vehicle):
            if n == 0:
                vehicle_transform = carla.Transform(carla.Location(x=-100, y=3, z=1), carla.Rotation(pitch=0, yaw=0, roll=0))
                vehicle_transform.rotation.yaw += 0
                position_list.append(vehicle_transform)
            else:
                vehicle_transform = carla.Transform(carla.Location(x=0, y=-100, z=1), carla.Rotation(pitch=0, yaw=0, roll=0))
                vehicle_transform.rotation.yaw += 90
                position_list.append(vehicle_transform)
        num_lost_vehicle = 0
        for n, position in enumerate(position_list):
            try:
                if n == 0:
                    self.vehicle_bp = random.choice(self.blueprint_library.filter("vehicle.audi.tt"))
                    print("self.blueprint_library is ", self.blueprint_library.filter("vehicle"))
                    self.vehicle_actor = self.world.spawn_actor(self.vehicle_bp, position)
                    self.vehicle_actor.set_light_state(light_state)
                    print("set_light_state is ", self.vehicle_actor.get_light_state())
                    # SetVehicleLightState(self.vehicle_actor, light_state)
                    self.custom_controller = VehiclePIDController(self.vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
                    self.vehicle_actor_list.append(self.vehicle_actor)
                    self.controller_list.append(self.custom_controller)

                    # 最初は全て衝突していない
                    self.vehicle_status[str(n - num_lost_vehicle)] = False

                else:
                    self.vehicle_bp = random.choice(self.blueprint_library.filter("vehicle.mustang.mustang"))
                    # print("self.blueprint_library is ", self.blueprint_library.filter("vehicle"))
                    self.vehicle_actor = self.world.spawn_actor(self.vehicle_bp, position)
                    self.vehicle_actor.set_light_state(light_state)
                    # SetVehicleLightState(self.vehicle_actor, light_state)
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
                controll_signal = self.controller_list[i].run_step(1000, waypoint)
                self.vehicle_actor_list[i].apply_control(controll_signal)
            else:
                controll_signal = carla.VehicleControl(brake=1.0)
                self.vehicle_actor_list[i].apply_control(controll_signal)
    def brake(self, vehicle_num = 0):
        controll_signal = carla.VehicleControl(brake=1.0)
        self.vehicle_actor_list[vehicle_num].apply_control(controll_signal)
        

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
        # self.world = client.load_world("/Game/Carla/Maps/Town03")

        # print("navigation location is ", self.world.get_random_location_from_navigation())

        # 設計図
        # LiDARのチャンネルなどの設定
        '''
        blueprint_library内に含まれる設計図のリストを作成
        '''
        blueprint_library = self.world.get_blueprint_library()

        # 観客視点とマップとwayPointの設定
        # self.spectator = self.world.get_spectator()
        # self.spectator.set_transform(carla.Transform(carla.Location(x=1.0, y=0, z=30), carla.Rotation(pitch=-70, yaw=-90)))

        self.map = self.world.get_map()
        waypoint_list = self.map.generate_waypoints(2.0)
        waypoint_tuple_list = self.map.get_topology()
        # self.visualize_waypoint(waypoint_tuple_list)

        # スポーンポイントを表示(test)
        # spawnpoint_list = self.world.get_map().get_spawn_points()
        # self.visualize_location(spawnpoint_list)

        '''
        IDによる設計図の選択.
        '''
        # Chose a vehicle blueprint
        self.GestVehicle = VehicleGenerator(self.world, self.map, blueprint_library, 2)
        # self.MainVehicle = VehicleGenerator(self.world, self.map, blueprint_library, 1)

        # self.world = self.MainVehicle.create_vehicle()
        self.world = self.GestVehicle.create_vehicle()

        # chose a walker blueprint
        self.Walker = WalkerGenerator(self.world, self.map, blueprint_library)

        self.world = self.Walker.create_walker()


        # 天気の変更
        weather = carla.WeatherParameters(
            # cloudiness=80.0,
            # precipitation=30.0,
            sun_altitude_angle=-90.0)

        self.world.set_weather(weather)

        print(self.world.get_weather())

        # Vehicle_pygame 画面設定
        pygame.init()
        self.vehicle_camera_H = 350
        self.vehicle_camera_W = 1000

        self.street_camera_H = 1080
        self.street_camera_W = 1920
        self.screen = pygame.display.set_mode((self.street_camera_W, self.street_camera_H))
        pygame.display.set_caption("Vehicle window")

        '''
        車両カメラの使用
        '''
        blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', str(self.vehicle_camera_W))
        blueprint.set_attribute('image_size_y', str(self.vehicle_camera_H))
        blueprint.set_attribute('fov', '60')
        transform = carla.Transform(carla.Location(x=-0.88, y = -0.42, z=1.11))
        self.vehicle_camera = self.world.spawn_actor(blueprint, transform, attach_to=self.GestVehicle.vehicle_actor_list[1])

        '''
        交差点カメラの使用
        '''
        brueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        brueprint.set_attribute('image_size_x', str(self.street_camera_W))
        brueprint.set_attribute('image_size_y', str(self.street_camera_H))
        brueprint.set_attribute('fov', '110')
        transform = carla.Transform(carla.Location(x=0, y=10, z=20), carla.Rotation(pitch=-40, yaw=-120))
        self.street_camera = self.world.spawn_actor(brueprint, transform)

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
        self.vehicle_camera.listen(lambda data: self.callback_camera_1(data))
        self.street_camera.listen(lambda data: self.callback_camera_2(data))
        start_time = time.time()
        while True:
            # 車両にカメラをくっつける
            # self.spectator.set_transform(self.camera_actor.get_transform())
            self.GestVehicle.vehicle_transport()
            temp_time = round(time.time() - start_time, 2)
            # print("walker location ", self.Walker.walker.get_location())
            walker_location = self.Walker.walker.get_location()
            vehicle_location = self.GestVehicle.vehicle_actor_list[1].get_location()

            
            #pygame
            # self.screen.fill((0, 0, 0))

            # 動作シナリオ
            # 車両を最初に止めておく(対 歩行者モード)
            self.GestVehicle.brake(0)

            # 歩行者を最初に止めておく(対 車両モード)
            # self.Walker.walker.set_location(carla.Location(x=100, y=100, z=100))
            # if vehicle_location.y > -15:
            #     self.GestVehicle.brake(1)

            # print("time is ", temp_time)
            if walker_location.x > -6:
                self.Walker.speed(0)

            if vehicle_location.y > -40:
                self.Walker.speed(2)

            if vehicle_location.y > -25:
                self.GestVehicle.brake(1)

    def callback_camera_1(self, data):
        pygame.display.update()
        # print("data is ", data)
        p1_camera = pygame.Rect(0, 0, self.vehicle_camera_W, self.vehicle_camera_H)

        data = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        data = np.reshape(data, (self.vehicle_camera_H, self.vehicle_camera_W, 4))
        data = data[:, :, :3]
        data = data[:, :, ::-1]
        surface = pygame.surfarray.make_surface(data.swapaxes(0, 1))
        self.screen.blit(surface, (0, 0), p1_camera)
        # self.screen.blit(surface, (0, 0), p1_camera)
        # print("rgb data is ", data.raw_data[0])
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

    def callback_camera_2(self, data):
        pygame.display.update()
        # print("data is ", data)
        p2_camera = pygame.Rect((self.vehicle_camera_W), 0, self.street_camera_W-self.vehicle_camera_W, self.street_camera_H-self.vehicle_camera_H)
        p3_camera = pygame.Rect(0, (self.vehicle_camera_H), self.vehicle_camera_W, (self.street_camera_H-self.vehicle_camera_H))
        p4_camera = pygame.Rect((self.vehicle_camera_W), (self.vehicle_camera_H), (self.street_camera_W-self.vehicle_camera_W), (self.street_camera_H-self.vehicle_camera_H))

        data = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
        data = np.reshape(data, (self.street_camera_H, self.street_camera_W, 4))
        data = data[:, :, :3]
        data = data[:, :, ::-1]
        # print("data is ", data)
        surface = pygame.surfarray.make_surface(data.swapaxes(0, 1))
        self.screen.blit(surface, ((self.vehicle_camera_W), 0), p2_camera)
        self.screen.blit(surface, (0, (self.vehicle_camera_H)), p3_camera)
        self.screen.blit(surface, ((self.vehicle_camera_W), (self.vehicle_camera_H)), p4_camera)
        # print("rgb data is ", data.raw_data[0])
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()


if __name__ == "__main__":
    World = World()
    World.update()
