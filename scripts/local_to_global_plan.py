# -----------------------------------------------------------------------------------------------
#  local_to_global_plan.py
#  Description: 
#        Chuyển local.plan (do offboard_Route_BVDN_TTHC.py ghi) -> global.plan (định dạng QGroundControl).

#        python3 local_to_global_plan.py                      # local.plan -> global.plan (cùng thư mục script)
#        python3 local_to_global_plan.py in.plan out.plan
#        python3 local_to_global_plan.py --track              # dùng quỹ đạo bay thực tế thay cho waypoint dự kiến
#        python3 local_to_global_plan.py --track --spacing 50 # khoảng cách tối thiểu giữa 2 điểm track (m)

#       Mở trong QGC: Plan View -> File -> Open -> chọn global.plan.

#       Hệ tọa độ:
#       local.plan : x = Bắc, y = Đông (m) so với điểm cất cánh; alt = độ cao so với điểm cất cánh (m).
#       global.plan: lat/lon (độ), độ cao tương đối so với home (MAV_FRAME_GLOBAL_RELATIVE_ALT).
#       Phép chiếu dùng giống PX4 (azimuthal equidistant, R = 6371 km) nên khớp với tọa độ PX4/QGC hiển thị.

#  Author:  REALTech Laboratory
#  Website: rltlab.io.vn
# -----------------------------------------------------------------------------------------------
#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys

EARTH_R = 6371000.0          # CONSTANTS_RADIUS_OF_EARTH của PX4

# MAVLink
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_AUTOPILOT_PX4 = 12
MAV_TYPE_QUADROTOR = 2


def reproject(lat0, lon0, north, east):
    """Local (Bắc, Đông) quanh gốc (lat0, lon0) -> (lat, lon). Giống MapProjection::reproject của PX4."""
    lat0_r, lon0_r = math.radians(lat0), math.radians(lon0)
    x_r, y_r = north / EARTH_R, east / EARTH_R
    c = math.hypot(x_r, y_r)
    if c < 1e-12:
        return lat0, lon0
    sin_c, cos_c = math.sin(c), math.cos(c)
    lat = math.asin(cos_c * math.sin(lat0_r) + x_r * sin_c * math.cos(lat0_r) / c)
    lon = lon0_r + math.atan2(y_r * sin_c, c * math.cos(lat0_r) * cos_c - x_r * math.sin(lat0_r) * sin_c)
    return math.degrees(lat), math.degrees(lon)


def home_global(plan):
    """Trả về (lat, lon, alt_amsl) của điểm cất cánh và nguồn dữ liệu."""
    ref = plan.get('reference') or {}
    if ref.get('xy_global'):
        # Gốc EKF của PX4 + vị trí lúc ARM trong hệ local NED
        x0, y0, z0 = ref.get('arm_origin_ned', [0.0, 0.0, 0.0])
        lat, lon = reproject(ref['ref_lat'], ref['ref_lon'], x0, y0)
        alt = ref['ref_alt'] - z0 if ref.get('z_global') else None
        return lat, lon, alt, 'PX4 EKF origin (ref_lat/ref_lon/ref_alt)'

    gz = plan.get('gazebo') or {}
    if gz.get('world_origin') and gz.get('home_enu'):
        # Dự phòng: gốc world Gazebo (spherical_coordinates) + vị trí spawn ENU
        o, (e, n, u) = gz['world_origin'], gz['home_enu']
        lat, lon = reproject(o['lat'], o['lon'], n, e)
        return lat, lon, o['elev'] + u, f"Gazebo world origin ({gz.get('world', '?')}) + home_enu"

    sys.exit('❌ local.plan không có reference hợp lệ (xy_global=false và không có thông tin gazebo).')


def simple_item(seq, command, frame, params, alt=None):
    item = {
        'type': 'SimpleItem',
        'autoContinue': True,
        'command': command,
        'doJumpId': seq,
        'frame': frame,
        'params': params,
    }
    if alt is not None:
        item.update({'Altitude': alt, 'AltitudeMode': 1, 'AMSLAltAboveTerrain': None})
    return item


def thin_track(track, spacing):
    """Giữ các điểm track cách nhau ít nhất `spacing` mét (theo phương ngang hoặc độ cao)."""
    out = []
    for p in track:
        if p.get('state') not in ('CRUISE', 'LAND'):
            continue
        if not out or math.hypot(p['x'] - out[-1]['x'], p['y'] - out[-1]['y']) >= spacing \
                or abs(p['alt'] - out[-1]['alt']) >= spacing:
            out.append(p)
    return out


def convert(plan, use_track=False, spacing=25.0):
    home_lat, home_lon, home_alt, src = home_global(plan)
    wps = {wp['name']: wp for wp in plan['waypoints']}
    speed = float(plan.get('cruise_speed', 5.0))

    def g(x, y):
        return reproject(home_lat, home_lon, x, y)

    items, seq = [], 1

    # 1. TAKEOFF tại home
    tk = wps['TAKEOFF']
    lat, lon = g(tk['x'], tk['y'])
    items.append(simple_item(seq, MAV_CMD_NAV_TAKEOFF, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             [0, 0, 0, None, lat, lon, tk['alt']], tk['alt']))
    seq += 1

    # 2. Tốc độ bay ngang giống script offboard
    items.append(simple_item(seq, MAV_CMD_DO_CHANGE_SPEED, MAV_FRAME_MISSION,
                             [1, speed, -1, 0, 0, 0, 0]))
    seq += 1

    # 3. Các waypoint giữa đường
    if use_track:
        mids = thin_track(plan.get('track', []), spacing)
        if not mids:
            print('⚠️  track rỗng, dùng waypoint dự kiến.')
    else:
        mids = []
    if not mids:
        mids = [wp for wp in plan['waypoints'] if wp['cmd'] == 'WAYPOINT']
    for p in mids:
        lat, lon = g(p['x'], p['y'])
        items.append(simple_item(seq, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 [0, 0, 0, None, lat, lon, p['alt']], p['alt']))
        seq += 1

    # 4. LAND tại đích (PX4 hạ thẳng đứng tới khi chạm đất, độ cao của item không dùng)
    ld = wps['LAND']
    lat, lon = g(ld['x'], ld['y'])
    items.append(simple_item(seq, MAV_CMD_NAV_LAND, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             [0, 0, 0, None, lat, lon, 0], 0))

    qgc = {
        'fileType': 'Plan',
        'version': 1,
        'groundStation': 'QGroundControl',
        'geoFence': {'circles': [], 'polygons': [], 'version': 2},
        'rallyPoints': {'points': [], 'version': 2},
        'mission': {
            'version': 2,
            'firmwareType': MAV_AUTOPILOT_PX4,
            'vehicleType': MAV_TYPE_QUADROTOR,
            'cruiseSpeed': speed,
            'hoverSpeed': speed,
            'globalPlanAltitudeMode': 1,
            'plannedHomePosition': [home_lat, home_lon, home_alt if home_alt is not None else 0],
            'items': items,
        },
    }
    return qgc, (home_lat, home_lon, home_alt, src)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='Chuyển local.plan -> global.plan cho QGroundControl')
    ap.add_argument('input', nargs='?', default=os.path.join(here, 'local.plan'))
    ap.add_argument('output', nargs='?', default=os.path.join(here, 'global.plan'))
    ap.add_argument('--track', action='store_true', help='dùng quỹ đạo bay thực tế (track) làm waypoint')
    ap.add_argument('--spacing', type=float, default=25.0, help='khoảng cách tối thiểu giữa điểm track (m)')
    a = ap.parse_args()

    with open(a.input, encoding='utf-8') as f:
        plan = json.load(f)
    if plan.get('fileType') != 'LocalPlan':
        sys.exit(f'❌ {a.input} không phải file LocalPlan.')

    qgc, (lat, lon, alt, src) = convert(plan, a.track, a.spacing)
    with open(a.output, 'w', encoding='utf-8') as f:
        json.dump(qgc, f, indent=4)

    print(f'Home: {lat:.7f}, {lon:.7f}, AMSL {alt if alt is None else round(alt, 2)} m  [{src}]')
    for it in qgc['mission']['items']:
        p = it['params']
        name = {16: 'WAYPOINT', 21: 'LAND', 22: 'TAKEOFF', 178: 'CHANGE_SPEED'}[it['command']]
        if it['command'] == MAV_CMD_DO_CHANGE_SPEED:
            print(f"  {it['doJumpId']:>3} {name:<12} {p[1]} m/s")
        else:
            print(f"  {it['doJumpId']:>3} {name:<12} {p[4]:.7f}, {p[5]:.7f}, alt {p[6]:.2f} m (rel home)")
    print(f'✅ Đã ghi {a.output} (result của chuyến bay: {plan.get("result")})')


if __name__ == '__main__':
    main()
