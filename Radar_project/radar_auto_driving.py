#!/usr/bin/env python
# -*- coding: utf-8 -*-
import carla
from carla import Transform, Location, Rotation

import random
import time
import math

class VehicleGenerator():
    def __init__(self, world, vehicle_bp, number_vehicle):
        self.number_vehicle = number_vehicle
        self.vehicle_bp = vehicle_bp
        self.world = world

    def create_vehicle(self):
        position_list = []
        vehicle_actor_list = []
        for n in range(self.number_vehicle):
            #spawn_point = carla.Transform()
            #spawn_point.location = self.world.get_random_location_from_navigation()
            vehicle_transform = random.choice(self.world.get_map().get_spawn_points())
            # print("transform is ", vehicle_transform)
            position_list.append(vehicle_transform)

        for n, position in enumerate(position_list):
            print("position is ", position)
            self.vehicle_actor = self.world.spawn_actor(self.vehicle_bp, position)
            # self.vehicle_actor.set_autopilot(True)

            #location = self.vehicle_actor.get_location()
            #location.z += 1
            #self.vehicle_actor.set_location(location)

            vehicle_actor_list.append(self.vehicle_actor)

        if self.number_vehicle == 1:
            return self.world, vehicle_actor_list[0]
        else:
            return self.world, vehicle_actor_list



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

        '''
        IDによる設計図の選択.
        '''
        # Chose a vehicle blueprint
        vehicle_bp = random.choice(blueprint_library.filter("vehicle.**.*"))
        self.world, self.gest_vehicle_act_list = VehicleGenerator(self.world, vehicle_bp, 10).create_vehicle()
        self.world, self.vehicle_actor = VehicleGenerator(self.world, vehicle_bp, 1).create_vehicle()

        self.camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', '1920')
        self.camera_bp.set_attribute('image_size_y', '1080')
        self.camera_bp.set_attribute('fov', '110')

        self.radar_bp = self.world.get_blueprint_library().find("sensor.other.radar")
        self.radar_bp.set_attribute('horizontal_fov', str(90))
        self.radar_bp.set_attribute('vertical_fov', str(20))
        self.radar_bp.set_attribute('range', str(20))
        rad_location = carla.Location(x=2.0, z=1.0)
        rad_rotation = carla.Rotation(pitch=5)

        # アクターの設置場所の決定
        camera_transform = carla.Transform(carla.Location(x=-20, z=3), carla.Rotation(pitch=-10))
        self.camera_actor = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.vehicle_actor)

        self.rad_transform = carla.Transform(rad_location,rad_rotation)
        self.rad_ego = self.world.spawn_actor(self.radar_bp,self.rad_transform,attach_to=self.vehicle_actor, attachment_type=carla.AttachmentType.Rigid)

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
        #print("wayponit_tuple is ", waypoint_tuple_list)
        #print("vehicle transform is ", self.vehicle_actor.get_transform())
        #my_geolocation = self.map.transform_to_geolocation(self.vehicle_actor.get_location())
        #print("my_geolocation", my_geolocation)
        #info_map = self.map.to_opendrive()
        #print("info_map is ", info_map)

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
                color=carla.Color(r, g, b))


    def update(self):
        self.rad_ego.listen(lambda radar_data: self.rad_callback(radar_data))
        while True:
            self.spectator.set_transform(self.camera_actor.get_transform())
            for vehicle in self.gest_vehicle_act_list:
                waypoint = self.map.get_waypoint(vehicle.get_location())
                waypoint = random.choice(waypoint.next(0.6))
                vehicle.set_transform(waypoint.transform)

            waypoint = self.map.get_waypoint(self.vehicle_actor.get_location())
            waypoint = random.choice(waypoint.next(0.6))
            self.vehicle_actor.set_transform(waypoint.transform)

            time.sleep(0.001)


if __name__ == "__main__":
    World = World()
    World.update()
