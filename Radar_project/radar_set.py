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
import pcl
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

        # radar_bp
        self.radar_bp = self.world.get_blueprint_library().find("sensor.other.radar")
        self.radar_bp.set_attribute('horizontal_fov', str(9))
        self.radar_bp.set_attribute('vertical_fov', str(9))
        self.radar_bp.set_attribute('range', str(50))
        self.radar_bp.set_attribute('points_per_second', str(1500))
        self.rad_location = carla.Location(x=1, y=0, z=1.0)
        self.rad_rotation = carla.Rotation(pitch=0, yaw=0)
        self.rad_transform = carla.Transform(self.rad_location, self.rad_rotation)

    def create_vehicle(self):
        position_list = []
        self.vehicle_actor_list = []
        self.rad_actor_list = []
        self.controller_list = []
        for n in range(self.number_vehicle):
            # spawn_point = carla.Transform()
            # spawn_point.location = self.world.get_random_location_from_navigation()
            vehicle_transform = random.choice(self.world.get_map().get_spawn_points())
            # print("transform is ", vehicle_transform)
            position_list.append(vehicle_transform)
        num_lost_vehicle = 0
        for n, position in enumerate(position_list):
            # print("position is ", position)
            try:
                self.vehicle_bp = random.choice(self.blueprint_library.filter("vehicle.**.*"))
                self.vehicle_actor = self.world.spawn_actor(self.vehicle_bp, position)
                self.custom_controller = VehiclePIDController(self.vehicle_actor, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})
                self.rad_ego = self.world.spawn_actor(self.radar_bp, self.rad_transform, attach_to=self.vehicle_actor, attachment_type=carla.AttachmentType.Rigid)
                self.vehicle_actor_list.append(self.vehicle_actor)
                self.controller_list.append(self.custom_controller)
                self.rad_actor_list.append(self.rad_ego)

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

    def vehicle_radar_callback(self, vehicle_num, radar_data):
        velocity_range = 7.5 # m/s
        current_rot = radar_data.transform.rotation
        collision_bool = []
        # print("vehicle_num is ", vehicle_num)
        # print("radar_data is ", radar_data, "vehicle_num is ", vehicle_num)
        for detect in radar_data:
            # print("detect is ", dete fst)
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            if (detect.depth < 3) and (-10 < azi < 10) and (-10 < alt < 10):
                collision_bool.append(True)
            else:
                continue

            # fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            # if (detect.velocity != 0) & (azi < 0):
            #     print("detect : velocity", detect.velocity, "destance", detect.depth, "azi", azi)
            # carla.Transform(
            #     carla.Location(),
            #     carla.Rotation(
            #         pitch=current_rot.pitch + alt,
            #         yaw=current_rot.yaw + azi,
            #         roll=current_rot.roll)).transform(fw_vec)

            # def clamp(min_v, max_v, value):
            #     return max(min_v, min(value, max_v))

            # norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            # r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            # g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            # b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            # #print("draw point is ", radar_data.transform.location)
            # # print("radar location is ", (radar_data.transform.location + fw_vec))
            # self.world.debug.draw_point(
            #     radar_data.transform.location + fw_vec,
            #     size=0.075,
            #     life_time=0.06,
            #     persistent_lines=False,
            #     color=carla.Color(r, g, b))

        if len(collision_bool) > 10:
            # print("vehicle__num", vehicle_num ,"detect depth ", detect.depth)
            self.vehicle_status[str(vehicle_num)] = True
        else:
            self.vehicle_status[str(vehicle_num)] = False

    def radar_listen(self):
        for i, radar_act in enumerate(self.rad_actor_list):
            radar_act.listen(lambda radar_data: self.vehicle_radar_callback(i, radar_data))


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
        self.world = client.load_world("/Game/Carla/Maps/Town01")

        # 設計図
        # LiDARのチャンネルなどの設定
        '''
        blueprint_library内に含まれる設計図のリストを作成
        '''
        blueprint_library = self.world.get_blueprint_library()

        # 観客視点とマップとwayPointの設定
        self.spectator = self.world.get_spectator()
        # self.spectator.set_transform(carla.Transform(carla.Location(x=90, y=131, z=10), carla.Rotation(pitch=-40, yaw=95)))
        self.spectator.set_transform(carla.Transform(carla.Location(x=90, y=131, z=10), carla.Rotation(pitch=-40, yaw=95)))
        self.map = self.world.get_map()

        waypoint_list = self.map.generate_waypoints(2.0)
        waypoint_tuple_list = self.map.get_topology()
        self.visualize_waypoint(waypoint_tuple_list)

        '''
        IDによる設計図の選択.
        '''
        # Chose a vehicle blueprint
        self.GestVehicle = VehicleGenerator(self.world, self.map, blueprint_library, 30)
        # self.MainVehicle = VehicleGenerator(self.world, self.map, blueprint_library, 1)

        self.world = self.GestVehicle.create_vehicle()
        # self.world = self.MainVehicle.create_vehicle()

        self.camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '1920')
        self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('fov', '200')

        self.radar_bp = self.world.get_blueprint_library().find("sensor.other.radar")
        self.radar_bp.set_attribute('horizontal_fov', str(9))
        self.radar_bp.set_attribute('vertical_fov', str(9))
        self.radar_bp.set_attribute('range', str(100))
        self.radar_bp.set_attribute('points_per_second', str(1500))
        rad_location = carla.Location(x=90.0, y=131, z=2.0)
        rad_rotation = carla.Rotation(pitch=-1, yaw=90)
        rad_box = carla.BoundingBox(rad_location, carla.Vector3D(x=0.5, y=0.5, z=0.5))

        # visualize radar
        self.world.debug.draw_box(
            rad_box,
            rad_rotation,
            thickness=0.1,
            color=carla.Color(0, 255, 255),
            life_time=-1.0
        )

        # アクターの設置場所の決定
        camera_transform = carla.Transform(carla.Location(x=95, y=150, z=1.0), carla.Rotation(pitch=0, yaw=95))
        self.camera_actor = self.world.spawn_actor(self.camera_bp, camera_transform)

        self.rad_transform = carla.Transform(rad_location,rad_rotation)
        self.rad_ego = self.world.spawn_actor(self.radar_bp,self.rad_transform, attachment_type=carla.AttachmentType.Rigid)

        # 天気の変更
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=30.0,
            sun_altitude_angle=70.0)

        self.world.set_weather(weather)

        print(self.world.get_weather())


    def visualize_waypoint(self, waypoint):
        for cuple_point in waypoint:
            #print("point is ", cuple_point[0])
            self.world.debug.draw_line(
                cuple_point[0].transform.location + carla.Location(z=0),
                cuple_point[1].transform.location + carla.Location(z=0),
                thickness=0.025,
                color=carla.Color(0, 255, 255))


    def rad_callback(self, radar_data):
        velocity_range = 7.5 # m/s
        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            # if (detect.velocity != 0) & (azi < 0):
            #     print("detect : velocity", detect.velocity, "destance", detect.depth, "azi", azi)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.world.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(255 - r, 255 - g, 255 - b))

    # def radar_clutering(self, radar_position):
    #     pointcloud = pcl.Pointclouod():

    # def carlaEventLoop(self, world):

    def update(self):
        self.rad_ego.listen(lambda radar_data: self.rad_callback(radar_data))
        # WorldThread = threading.Thread(target=self.carlaEventLoop, args=[self.world], daemon=True)
        # WorldThread.start()
        # pdb.set_trace()
        while True:
            # self.GestVehicle.radar_listen()
            self.GestVehicle.vehicle_transport()
            time.sleep(0.001)

            # self.world.tick()
        # GestThread = threading.Thread(target=self.GestVehicle.radar_listen())
        # GestThread.start()
        # MainThread = threading.Thread(target=self.MainVehicle.radar_listen())
        # MainThread.start()

if __name__ == "__main__":
    World = World()
    World.update()
