#!/usr/bin/env python
# -*- coding: utf-8 -*-
import carla
from carla import Transform, Location, Rotation

import random
import time
import math


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
        vehicle_bp = random.choice(blueprint_library.filter("vehicle.bmw.*"))

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
        '''
        actor の設置を行う.
        '''
        vehicle_transform = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle_actor = self.world.spawn_actor(vehicle_bp, vehicle_transform)
        self.vehicle_actor.set_autopilot(True)

        camera_transform = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-10))
        self.camera_actor = self.world.spawn_actor(self.camera_bp, camera_transform, attach_to=self.vehicle_actor)

        self.rad_transform = carla.Transform(rad_location,rad_rotation)
        self.rad_ego = self.world.spawn_actor(self.radar_bp,self.rad_transform,attach_to=self.vehicle_actor, attachment_type=carla.AttachmentType.Rigid)

        # アクターの処理
        location = self.vehicle_actor.get_location()
        location.z += 1
        self.vehicle_actor.set_location(location)
        #print("moved vehicle to %s" % location)

        # 天気の変更
        weather = carla.WeatherParameters(
            cloudiness=80.0,
            precipitation=30.0,
            sun_altitude_angle=70.0)

        self.world.set_weather(weather)

        print(self.world.get_weather())

        # マップとwayPointの設定
        self.spectator = self.world.get_spectator()
        map = self.world.get_map()


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
            time.sleep(0.01)

if __name__ == "__main__":
    World = World()
    World.update()
