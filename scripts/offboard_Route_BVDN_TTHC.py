# -----------------------------------------------------------------------------------------------
#  offboard_Route_BVDN_TTHC.py
#  Description: Chương trình thử nghiệm bay từ đỉnh tòa nhà Bệnh Viện Đa Khoa Đà Nẵng 
#               đến sân đỗ xe Trung tâm Hành Chính Thành phố Đà Nẵng
#  Author:  REALTech Laboratory
#  Website: rltlab.io.vn
# -----------------------------------------------------------------------------------------------
#!/usr/bin/env python3
import rclpy
import math
import os
import json
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class OffboardRouteControl(Node):
    def __init__(self):
        super().__init__('offboard_route_control_node')

        # 1. Khởi tạo giá trị bộ nhớ đệm trạng thái drone
        self.vehicle_status = VehicleStatus() 
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        # 2. KHỞI TẠO CÁC NHÓM LUỒNG (CALLBACK GROUPS) TRƯỚC
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.sensor_group = MutuallyExclusiveCallbackGroup()

        # 3. ĐỊNH NGHĨA CÁC CẤU HÌNH QOS CHUẨN
        # QoS dành riêng cho các lệnh gửi đi (Publisher)
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS dành riêng cho các cảm biến nhận về (Subscriber) - Đã ép về VOLATILE chống nghẽn
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 4. KHỞI TẠO PUBLISHERS (Dùng qos_profile_pub)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_pub)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_pub)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile_pub)

        # 5. KHỞI TẠO SUBSCRIBERS (Dùng qos_profile_sub và đưa vào sensor_group để chạy ngầm song song)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile_sub, callback_group=self.sensor_group
        )
        
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile_sub, callback_group=self.sensor_group
        )

        # 6. KHỞI TẠO VÒNG LẶP ĐIỀU KHIỂN (Gán riêng biệt vào timer_group)
        self.dt = 0.1  
        self.timer = self.create_timer(self.dt, self.timer_callback, callback_group=self.timer_group)
        
        self.offboard_setpoint_counter = 0
        self.flight_state = "TAKEOFF"
        self.state_timer = 0.0

        # 7. TỌA ĐỘ HÀNH TRÌNH (Gazebo world ENU: x=Đông, y=Bắc, z=Lên)
        #    Home phải trùng PX4_GZ_MODEL_POSE trong run_px4.sh
        self.HOME_ENU = (48.36, -575.63, 23.28)
        self.TARGET_ENU = (898.3, -309.3, 6.03)
        # Đổi sang hệ local NED của PX4 (gốc = home): X=Bắc, Y=Đông, Z=Xuống
        self.target_x = self.TARGET_ENU[1] - self.HOME_ENU[1]   # +266.33 m Bắc
        self.target_y = self.TARGET_ENU[0] - self.HOME_ENU[0]   # +849.94 m Đông
        self.cruise_z = -10.0                                     # cất cánh lên thêm 10m so với home
        self.cruise_speed = 8.0                                   # m/s tốc độ bay ngang
        self.route_len = math.hypot(self.target_x, self.target_y)
        self.target_yaw = math.atan2(self.target_y, self.target_x)  # mũi drone hướng về đích
        self.carrot_s = 0.0
        self.x0 = self.y0 = self.z0 = 0.0   # gốc thực tế, cập nhật lúc ARM

        # 8. GHI LỘ TRÌNH RA FILE local.plan (cùng thư mục với script)
        self.plan_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'local.plan')
        self.WORLD_NAME = '55000-HAICHAU'
        self.WORLD_ORIGIN = {'lat': 16.078691652164597, 'lon': 108.21459523792167, 'elev': 5.2}  # spherical_coordinates
        self.ref = None              # gốc EKF của PX4 (ref_lat/ref_lon/ref_alt)
        self.plan = None
        self.track_timer = 0.0
        self.track_period = 1.0      # s, chu kỳ lưu vị trí thực tế


    def status_callback(self, msg):
        # Cảnh báo khi PX4 tự thoát Offboard (failsafe RC loss -> RTL, ...)
        if (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.flight_state not in ("LANDING", "DONE")):
            self.get_logger().error(
                f"⛔ PX4 đã thoát OFFBOARD (nav_state={msg.nav_state}) khi đang ở bước {self.flight_state}! "
                "Thường do failsafe 'Manual control lost' -> RTL. Xem COM_RCL_EXCEPT.")
        self.vehicle_status = msg

    def local_position_callback(self, msg):
        """Hàm này sẽ hứng dữ liệu sống từ DDS phát ra liên tục 50Hz"""
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        self.ref = {'ref_lat': msg.ref_lat, 'ref_lon': msg.ref_lon, 'ref_alt': float(msg.ref_alt),
                    'xy_global': bool(msg.xy_global), 'z_global': bool(msg.z_global)}

    # ------------------------- GHI FILE local.plan -------------------------
    def rel_pos(self):
        """Vị trí hiện tại so với điểm cất cánh: X Bắc, Y Đông, độ cao (m, dương = lên)"""
        return (round(self.drone_x - self.x0, 3), round(self.drone_y - self.y0, 3), round(self.z0 - self.drone_z, 3))

    def init_plan(self):
        """Tạo toàn bộ waypoint dự kiến của hành trình (gọi lúc ARM, khi đã biết gốc)"""
        alt = -self.cruise_z
        land_ground_alt = self.TARGET_ENU[2] - self.HOME_ENU[2]   # mặt đất đích so với điểm cất cánh
        ref = dict(self.ref or {})
        ref['arm_origin_ned'] = [self.x0, self.y0, self.z0]
        if ref.get('z_global'):
            ref['home_alt_amsl'] = ref['ref_alt'] - self.z0
        self.plan = {
            'fileType': 'LocalPlan',
            'version': 1,
            'created': datetime.now().isoformat(timespec='seconds'),
            'frame': 'Local NED so với điểm cất cánh: x = Bắc (m), y = Đông (m), alt = độ cao so với điểm cất cánh (m, dương = lên)',
            'reference': ref,
            'gazebo': {'world': self.WORLD_NAME, 'world_origin': self.WORLD_ORIGIN,
                       'home_enu': list(self.HOME_ENU), 'target_enu': list(self.TARGET_ENU)},
            'cruise_speed': self.cruise_speed,
            'waypoints': [
                {'seq': 0, 'name': 'HOME',     'cmd': 'HOME',     'x': 0.0, 'y': 0.0, 'alt': 0.0},
                {'seq': 1, 'name': 'TAKEOFF',  'cmd': 'TAKEOFF',  'x': 0.0, 'y': 0.0, 'alt': alt},
                {'seq': 2, 'name': 'TARGET',   'cmd': 'WAYPOINT', 'x': round(self.target_x, 3),
                 'y': round(self.target_y, 3), 'alt': alt},
                {'seq': 3, 'name': 'LAND',     'cmd': 'LAND',     'x': round(self.target_x, 3),
                 'y': round(self.target_y, 3), 'alt': round(land_ground_alt, 3)},
            ],
            'track': [],
            'result': 'IN_PROGRESS',
        }
        for wp in self.plan['waypoints']:
            wp['status'] = 'PLANNED'
        self.mark_reached('HOME')

    def mark_reached(self, name):
        """Lưu vị trí thực tế khi drone hoàn thành 1 waypoint"""
        if self.plan is None:
            return
        x, y, alt = self.rel_pos()
        for wp in self.plan['waypoints']:
            if wp['name'] == name:
                wp['status'] = 'REACHED'
                wp['actual'] = {'x': x, 'y': y, 'alt': alt, 'time': datetime.now().isoformat(timespec='seconds')}
        self.save_plan()

    def record_track(self):
        if self.plan is None:
            return
        self.track_timer += self.dt
        if self.track_timer < self.track_period:
            return
        self.track_timer = 0.0
        x, y, alt = self.rel_pos()
        self.plan['track'].append({'t': datetime.now().isoformat(timespec='seconds'),
                                   'state': self.flight_state, 'x': x, 'y': y, 'alt': alt})
        self.save_plan()

    def save_plan(self):
        if self.plan is None:
            return
        try:
            tmp = self.plan_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(self.plan, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self.plan_path)  
        except OSError as e:
            self.get_logger().error(f"Không ghi được {self.plan_path}: {e}")

    def timer_callback(self):
        # Tạo xung nhịp Offboard để PX4 không thoát Offboard (failsafe) khi chưa có lệnh nào gửi đi
        if self.offboard_setpoint_counter < 10:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0)
            self.offboard_setpoint_counter += 1
            return

        # Thực thi lệnh ARM tự động lên PX4
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.offboard_setpoint_counter += 1
            # Lấy vị trí lúc ARM làm gốc: local z=0 của EKF không chắc trùng mặt đất (nóc nhà)
            self.x0, self.y0, self.z0 = self.drone_x, self.drone_y, self.drone_z
            self.get_logger().info(
                f"[Gốc] Vị trí lúc ARM: X={self.x0:.2f}, Y={self.y0:.2f}, Z={self.z0:.2f} (local NED)")
            self.init_plan()
            self.get_logger().info(f"[Plan] Lưu waypoint vào {self.plan_path}")

        # Sau khi đã gửi lệnh LAND, ngừng gửi heartbeat Offboard để PX4 tự hạ cánh
        if self.flight_state in ("LANDING", "DONE"):
            if self.flight_state == "LANDING":
                self.record_track()   # tiếp tục lưu quỹ đạo trong lúc PX4 tự hạ cánh
            if (self.flight_state == "LANDING"
                    and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                self.flight_state = "DONE"
                self.mark_reached('LAND')
                self.plan['result'] = 'LANDED'
                self.save_plan()
                self.get_logger().info(
                    f"✅ Đã hạ cánh & disarm tại N={self.drone_x - self.x0:.2f}, E={self.drone_y - self.y0:.2f} "
                    f"(Gazebo ENU ≈ {self.HOME_ENU[0] + self.drone_y - self.y0:.2f}, "
                    f"{self.HOME_ENU[1] + self.drone_x - self.x0:.2f})")
            return

        self.publish_offboard_control_mode()
        self.state_timer += self.dt
        self.record_track()

        # --- IN LOG TỌA ĐỘ SỐNG LÊN TERMINAL (1 GIÂY MỘT LẦN) ---
        self.get_logger().info(
            f"📍 [{self.flight_state}] X (Bắc): {self.drone_x - self.x0:.2f}m | Y (Đông): {self.drone_y - self.y0:.2f}m | "
            f"Cao so với điểm cất cánh: {self.z0 - self.drone_z:.2f}m | nav_state={self.vehicle_status.nav_state}",
            throttle_duration_sec=1.0
        )

        # --- QUỸ ĐẠO: CẤT CÁNH +10m -> BAY THẲNG TỚI ĐÍCH -> HẠ CÁNH ---
        if self.flight_state == "TAKEOFF":
            # Giai đoạn 1: Cất cánh thẳng đứng lên +10m so với home (NED: Z=-10)
            self.publish_trajectory_setpoint(self.x0, self.y0, self.z0 + self.cruise_z, self.target_yaw)
            climbed = self.z0 - self.drone_z            # độ cao đã leo so với điểm cất cánh
            reached = climbed >= -self.cruise_z - 0.5   # chỉ cần đạt >= 9.5m, vượt lên cũng tính
            if (reached and self.state_timer > 3.0) or self.state_timer > 20.0:
                if not reached:
                    self.get_logger().warn(f"[Hành trình] Hết 20s mới leo được {climbed:.2f}m, vẫn bay tiếp.")
                self.flight_state = "CRUISE"
                self.state_timer = 0.0
                self.mark_reached('TAKEOFF')
                self.get_logger().info(
                    f"[Hành trình] Đã leo {climbed:.1f}m. Bay tới đích "
                    f"(N={self.target_x:.2f}, E={self.target_y:.2f}), quãng đường {self.route_len:.1f}m")

        elif self.flight_state == "CRUISE":
            # Giai đoạn 2: Điểm đặt "cà rốt" tiến dần dọc đường thẳng với vận tốc cruise_speed
            # (tránh nhảy setpoint ~890m một lần khiến drone nghiêng gắt)
            self.carrot_s = min(self.carrot_s + self.cruise_speed * self.dt, self.route_len)
            ratio = self.carrot_s / self.route_len
            self.publish_trajectory_setpoint(self.x0 + self.target_x * ratio, self.y0 + self.target_y * ratio,
                                             self.z0 + self.cruise_z, self.target_yaw)
            dist = math.hypot(self.drone_x - self.x0 - self.target_x, self.drone_y - self.y0 - self.target_y)
            if self.carrot_s >= self.route_len and dist <= 2.0:
                self.flight_state = "LAND"
                self.state_timer = 0.0
                self.mark_reached('TARGET')
                self.get_logger().info(f"[Hành trình] Đã tới trên điểm đích (sai số {dist:.2f}m). Bắt đầu hạ cánh!")

        elif self.flight_state == "LAND":
            # Giai đoạn 3: Giữ vị trí ổn định 2s rồi gửi NAV_LAND, PX4 tự hạ và tự disarm khi chạm đất
            self.publish_trajectory_setpoint(self.x0 + self.target_x, self.y0 + self.target_y,
                                             self.z0 + self.cruise_z, self.target_yaw)
            if self.state_timer > 2.0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.flight_state = "LANDING"
                self.state_timer = 0.0
                if self.plan is not None:
                    self.plan['track'].append({'t': datetime.now().isoformat(timespec='seconds'),
                                               'state': 'LAND_CMD', **dict(zip(('x', 'y', 'alt'), self.rel_pos()))})
                    self.save_plan()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardRouteControl()
    
    # Khởi tạo Executor chạy đa luồng thật sự song song (1 luồng kiểm soát bay, 1 luồng hứng cảm biến)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.plan is not None:
            if node.plan['result'] == 'IN_PROGRESS':
                node.plan['result'] = f'ABORTED_AT_{node.flight_state}'
            node.save_plan()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()