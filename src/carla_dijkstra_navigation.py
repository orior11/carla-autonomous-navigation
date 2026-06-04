#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CARLA 0.9.x - Custom navigation A -> B using a hand-written Dijkstra planner.

Highlights / fixes integrated:
  * Correct route reconstruction (forward order, no reversed segments,
    no duplicated junction nodes, includes both start and goal points).
  * _densify_segment always closes the segment on the exit waypoint.
  * Distance-based pure-pursuit look-ahead (no fixed index) -> the car tracks
    the lane tightly instead of cutting corners into roadside signs/poles.
  * Automatic speed reduction before curves (curvature look-ahead).
  * Longitudinal controller brakes when overspeed, not just coasts.
  * Empty-route guard (raises a clear error if no path is found).
  * Robust connection handling: host/port are configurable from the command
    line, printed at startup, and a clear message is shown if the port is
    wrong / the server is not reachable.

Run:
    python carla_dijkstra_navigation.py
    python carla_dijkstra_navigation.py --host 127.0.0.1 --port 2000
    python carla_dijkstra_navigation.py --a 0 --b 100 --speed 25

IMPORTANT - matching the CARLA port:
    The --port value MUST equal the RPC port your CARLA server is listening on.
    By default CARLA listens on 2000. If you launched it with a different port,
    e.g. "./CarlaUE4.sh -carla-rpc-port=3000", run this script with --port 3000.
"""

import sys
import math
import heapq
import random
import argparse
from collections import deque

import numpy as np

try:
    import carla
except ImportError:
    sys.exit("ERROR: could not import the 'carla' module. Make sure the CARLA "
             "PythonAPI matching your server version is installed / on PYTHONPATH.")

# ---------------------------------------------------------------------------
# DEFAULT CONFIG (can be overridden on the command line)
# ---------------------------------------------------------------------------
HOST = "127.0.0.1"
PORT = 2000
TIMEOUT = 60.0

SPAWN_INDEX_A = 0
SPAWN_INDEX_B = 100

SAMPLING_RESOLUTION = 2.0       # meters between densified waypoints
TARGET_SPEED_KMH = 22.0         # cruising speed on straights
ARRIVAL_RADIUS = 3.0            # "arrived" distance to destination (m)
WAYPOINT_REACHED_RADIUS = 3.0   # drop a waypoint once we are this close (m)
VEHICLE_BLUEPRINT = "vehicle.tesla.model3"
FIXED_DELTA = 0.05
DRAW_ROUTE = True

# Pure-pursuit look-ahead: scales with speed but stays small so the car does
# not cut corners. Distance = clamp(speed_mps * K, MIN, MAX).
LOOKAHEAD_K = 0.6
LOOKAHEAD_MIN = 3.0
LOOKAHEAD_MAX = 7.0


# ===========================================================================
# DIJKSTRA ROUTE PLANNER (graph built from map.get_topology())
# ===========================================================================
class DijkstraRoutePlanner:
    def __init__(self, carla_map, sampling_resolution=SAMPLING_RESOLUTION):
        self._map = carla_map
        self._resolution = sampling_resolution
        self._graph = {}            # node_id -> {neighbor_id: (weight, [waypoints])}
        self._id_to_location = {}   # node_id -> carla.Location
        self._build_graph()

    @staticmethod
    def _node_id(waypoint):
        # A node is its location rounded to whole meters, so endpoints shared
        # between segments collapse onto the same node (connects junctions).
        loc = waypoint.transform.location
        return (round(loc.x), round(loc.y), round(loc.z))

    def _densify_segment(self, entry_wp, exit_wp):
        # Walk the real road geometry from entry to exit using next().
        end_loc = exit_wp.transform.location
        path = [entry_wp]
        if entry_wp.transform.location.distance(end_loc) > self._resolution:
            nxt = entry_wp.next(self._resolution)
            if nxt:
                w = nxt[0]
                guard = 0
                while (w.transform.location.distance(end_loc) > self._resolution
                       and guard < 10000):
                    path.append(w)
                    nxt = w.next(self._resolution)
                    if not nxt:
                        break
                    w = nxt[0]
                    guard += 1
        # Always close the segment on the exit waypoint (fixes missing endpoint).
        path.append(exit_wp)
        return path

    @staticmethod
    def _path_length(waypoints):
        return sum(a.transform.location.distance(b.transform.location)
                   for a, b in zip(waypoints[:-1], waypoints[1:]))

    def _build_graph(self):
        topology = self._map.get_topology()
        if not topology:
            raise RuntimeError("map.get_topology() returned no segments.")
        for entry_wp, exit_wp in topology:
            n1, n2 = self._node_id(entry_wp), self._node_id(exit_wp)
            self._id_to_location[n1] = entry_wp.transform.location
            self._id_to_location[n2] = exit_wp.transform.location
            dense = self._densify_segment(entry_wp, exit_wp)
            weight = self._path_length(dense)
            self._graph.setdefault(n1, {})
            self._graph.setdefault(n2, {})
            existing = self._graph[n1].get(n2)
            if existing is None or weight < existing[0]:
                self._graph[n1][n2] = (weight, dense)
        print("[Planner] Graph built: {} nodes.".format(len(self._graph)))

    def _nearest_node(self, location):
        return min(self._id_to_location,
                   key=lambda k: self._id_to_location[k].distance(location))

    def trace_route(self, start_loc, end_loc):
        start_id = self._nearest_node(start_loc)
        goal_id = self._nearest_node(end_loc)

        # ---- Dijkstra (lazy; first pop of goal is optimal) ----
        dist, prev, pq = {start_id: 0.0}, {}, [(0.0, start_id)]
        while pq:
            cost, u = heapq.heappop(pq)
            if u == goal_id:
                break
            if cost > dist.get(u, float("inf")):
                continue  # stale heap entry
            for v, (w, _path) in self._graph.get(u, {}).items():
                new_cost = cost + w
                if new_cost < dist.get(v, float("inf")):
                    dist[v], prev[v] = new_cost, u
                    heapq.heappush(pq, (new_cost, v))

        # No path found.
        if goal_id != start_id and goal_id not in prev:
            return []

        # ---- Reconstruct FORWARD, prepending each edge ----
        # Process goal -> start. Keep the last edge whole (so the goal point is
        # included); for every earlier edge drop its shared junction node to
        # avoid duplicating it where edges meet.
        route, curr, first = [], goal_id, True
        while curr in prev:
            _, dense = self._graph[prev[curr]][curr]
            route = (dense if first else dense[:-1]) + route
            first = False
            curr = prev[curr]
        return route


# ===========================================================================
# PID CONTROLLERS
# ===========================================================================
class PIDLongitudinalController:
    def __init__(self, kp=1.0, ki=0.05, kd=0.0, dt=FIXED_DELTA):
        self._kp, self._ki, self._kd, self._dt = kp, ki, kd, dt
        self._errors = deque(maxlen=10)

    def run_step(self, target_speed_kmh, current_speed_kmh):
        error = target_speed_kmh - current_speed_kmh
        self._errors.append(error)
        if len(self._errors) >= 2:
            derivative = (self._errors[-1] - self._errors[-2]) / self._dt
            integral = sum(self._errors) * self._dt
        else:
            derivative = integral = 0.0
        # Output in [-1, 1]; positive -> throttle, negative -> brake.
        return float(np.clip(
            self._kp * error + self._kd * derivative + self._ki * integral,
            -1.0, 1.0))


class PIDLateralController:
    def __init__(self, kp=1.5, ki=0.05, kd=0.4, dt=FIXED_DELTA):
        self._kp, self._ki, self._kd, self._dt = kp, ki, kd, dt
        self._errors = deque(maxlen=10)

    def run_step(self, vehicle_transform, target_location):
        # Error = signed angle between the car's forward vector and the
        # direction to the target waypoint. Sign from the 2D cross product.
        loc = vehicle_transform.location
        yaw = math.radians(vehicle_transform.rotation.yaw)
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        to_target = np.array([target_location.x - loc.x,
                              target_location.y - loc.y])
        norm = np.linalg.norm(to_target)
        if norm < 1e-6:
            error = 0.0
        else:
            to_target /= norm
            dot = float(np.clip(np.dot(forward, to_target), -1.0, 1.0))
            error = math.acos(dot)
            cross = forward[0] * to_target[1] - forward[1] * to_target[0]
            if cross < 0.0:
                error = -error
        self._errors.append(error)
        if len(self._errors) >= 2:
            derivative = (self._errors[-1] - self._errors[-2]) / self._dt
            integral = sum(self._errors) * self._dt
        else:
            derivative = integral = 0.0
        return float(np.clip(
            self._kp * error + self._kd * derivative + self._ki * integral,
            -1.0, 1.0))


class VehicleController:
    def __init__(self, dt=FIXED_DELTA):
        self._lon = PIDLongitudinalController(dt=dt)
        self._lat = PIDLateralController(dt=dt)
        self._last_steer = 0.0

    @staticmethod
    def speed_kmh(vehicle):
        v = vehicle.get_velocity()
        return 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

    def run_step(self, vehicle, target_location, target_speed_kmh):
        speed = self.speed_kmh(vehicle)
        accel = self._lon.run_step(target_speed_kmh, speed)
        steer = self._lat.run_step(vehicle.get_transform(), target_location)

        # Smooth steering but allow a real response (was previously too slow).
        max_delta = 0.2
        steer = float(np.clip(steer, self._last_steer - max_delta,
                              self._last_steer + max_delta))
        self._last_steer = steer

        control = carla.VehicleControl()
        control.steer = float(np.clip(steer, -1.0, 1.0))
        if accel >= 0.0:
            control.throttle = float(np.clip(accel, 0.0, 1.0))
            control.brake = 0.0
        else:
            # Overspeed -> actually brake instead of just coasting.
            control.throttle = 0.0
            control.brake = float(np.clip(-accel, 0.0, 1.0))
        return control


# ===========================================================================
# HELPERS
# ===========================================================================
def update_spectator(spectator, vehicle, distance=6.0, height=3.0, pitch=-15.0):
    # Chase camera: behind and above the car, looking where it heads.
    vt = vehicle.get_transform()
    f = vt.get_forward_vector()
    loc = carla.Location(x=vt.location.x - distance * f.x,
                         y=vt.location.y - distance * f.y,
                         z=vt.location.z + height)
    rot = carla.Rotation(pitch=pitch, yaw=vt.rotation.yaw, roll=0.0)
    spectator.set_transform(carla.Transform(loc, rot))


def draw_route(world, route, life_time=120.0):
    for i in range(len(route) - 1):
        a = route[i].transform.location + carla.Location(z=0.5)
        b = route[i + 1].transform.location + carla.Location(z=0.5)
        world.debug.draw_line(a, b, thickness=0.2,
                              color=carla.Color(0, 255, 0),
                              life_time=life_time)


def pick_lookahead_target(route_queue, vehicle_loc, lookahead_dist):
    # First waypoint at least lookahead_dist away (pure-pursuit target).
    target = route_queue[0]
    for wp in route_queue:
        if vehicle_loc.distance(wp.transform.location) >= lookahead_dist:
            return wp
        target = wp
    return target


def upcoming_curvature(route_queue, n=6):
    # Largest heading change (radians) over the next few waypoints.
    pts = list(route_queue)[:n + 1]
    if len(pts) < 3:
        return 0.0
    max_ang = 0.0
    for i in range(len(pts) - 2):
        a = pts[i].transform.location
        b = pts[i + 1].transform.location
        c = pts[i + 2].transform.location
        v1 = np.array([b.x - a.x, b.y - a.y])
        v2 = np.array([c.x - b.x, c.y - b.y])
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        ang = math.acos(float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)))
        max_ang = max(max_ang, ang)
    return max_ang


def speed_for_curve(curvature_rad):
    # Slow down before bends to avoid understeering wide off the lane.
    if curvature_rad > 0.6:
        return 10.0
    if curvature_rad > 0.3:
        return 15.0
    return TARGET_SPEED_KMH


# ===========================================================================
# ARGUMENTS
# ===========================================================================
def parse_args():
    p = argparse.ArgumentParser(description="CARLA Dijkstra navigation A -> B.")
    p.add_argument("--host", default=HOST, help="CARLA server host (default %(default)s)")
    p.add_argument("--port", type=int, default=PORT, help="CARLA RPC port (default %(default)s)")
    p.add_argument("--a", type=int, default=SPAWN_INDEX_A, help="Start spawn-point index")
    p.add_argument("--b", type=int, default=SPAWN_INDEX_B, help="Destination spawn-point index")
    p.add_argument("--speed", type=float, default=TARGET_SPEED_KMH, help="Cruise speed km/h")
    p.add_argument("--vehicle", default=VEHICLE_BLUEPRINT, help="Vehicle blueprint id")
    p.add_argument("--no-draw", action="store_true", help="Do not draw the route")
    return p.parse_args()


# ===========================================================================
# MAIN
# ===========================================================================
def main():
    global TARGET_SPEED_KMH  # speed_for_curve reads this
    args = parse_args()
    TARGET_SPEED_KMH = args.speed
    draw = not args.no_draw

    actor_list = []
    original_settings = None
    world = None
    collisions = []

    try:
        # ----- connect (with a clear message if the port is wrong) ---------
        print("[Setup] Connecting to CARLA at {}:{} ...".format(args.host, args.port))
        client = carla.Client(args.host, args.port)
        client.set_timeout(TIMEOUT)
        try:
            world = client.get_world()
        except RuntimeError as err:
            sys.exit(
                "\n[ERROR] Could not reach the CARLA server at {}:{}.\n"
                "        ({})\n"
                "        Check that:\n"
                "          1. The CARLA simulator is running.\n"
                "          2. --port matches the server's RPC port "
                "(default 2000, or whatever you passed to "
                "-carla-rpc-port=...).\n".format(args.host, args.port, err))

        carla_map = world.get_map()
        spectator = world.get_spectator()
        print("[Setup] Connected. Map: {}".format(carla_map.name))

        # ----- synchronous fixed-delta mode --------------------------------
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(settings)

        # ----- resolve A / B and spawn -------------------------------------
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("This map has no spawn points.")
        idx_a = args.a % len(spawn_points)
        idx_b = args.b % len(spawn_points)
        start_tf = spawn_points[idx_a]
        loc_b = spawn_points[idx_b].location

        bp = world.get_blueprint_library().find(args.vehicle)
        if bp.has_attribute("color"):
            bp.set_attribute("color", random.choice(
                bp.get_attribute("color").recommended_values))
        vehicle = world.try_spawn_actor(bp, start_tf)
        if vehicle is None:
            start_tf = carla.Transform(
                start_tf.location + carla.Location(z=0.5), start_tf.rotation)
            vehicle = world.try_spawn_actor(bp, start_tf)
        if vehicle is None:
            raise RuntimeError("Could not spawn the vehicle at spawn index {}."
                               .format(idx_a))
        actor_list.append(vehicle)
        print("[Setup] Vehicle spawned at Point A (index {}).".format(idx_a))

        # ----- collision sensor (diagnostics) ------------------------------
        col_bp = world.get_blueprint_library().find("sensor.other.collision")
        col_sensor = world.spawn_actor(col_bp, carla.Transform(),
                                       attach_to=vehicle)
        actor_list.append(col_sensor)
        col_sensor.listen(lambda e: collisions.append(
            getattr(e.other_actor, "type_id", "unknown")))

        # Settle physics and snap the camera onto the car before it moves.
        for _ in range(10):
            world.tick()
            update_spectator(spectator, vehicle)

        # ----- PHASE 1: plan with Dijkstra ---------------------------------
        print("[Planner] Searching shortest path A -> B (this may take a moment) ...")
        planner = DijkstraRoutePlanner(world.get_map(), SAMPLING_RESOLUTION)
        route = planner.trace_route(vehicle.get_location(), loc_b)
        if not route:
            raise RuntimeError("Dijkstra found no path between A and B.")
        print("[Planner] Route found: {} waypoints, ~{:.0f} m.".format(
            len(route), DijkstraRoutePlanner._path_length(route)))
        if draw:
            draw_route(world, route)
            print("[Planner] Route drawn in green inside the simulator.")

        # ----- PHASE 2: drive ----------------------------------------------
        print("[Drive] Starting navigation ...")
        controller = VehicleController(dt=FIXED_DELTA)
        route_q = deque(route)
        destination = route[-1].transform.location
        total = len(route)
        last_pct = -10

        while True:
            world.tick()
            update_spectator(spectator, vehicle)
            veh_loc = vehicle.get_location()

            if veh_loc.distance(destination) < ARRIVAL_RADIUS:
                break

            # Drop reached waypoints.
            while (route_q and
                   veh_loc.distance(route_q[0].transform.location)
                   < WAYPOINT_REACHED_RADIUS):
                route_q.popleft()
            if not route_q:
                break

            # Speed-dependent look-ahead + curve slowdown.
            speed_mps = VehicleController.speed_kmh(vehicle) / 3.6
            lookahead = float(np.clip(speed_mps * LOOKAHEAD_K,
                                      LOOKAHEAD_MIN, LOOKAHEAD_MAX))
            target_wp = pick_lookahead_target(route_q, veh_loc, lookahead)
            target_speed = speed_for_curve(upcoming_curvature(route_q))

            vehicle.apply_control(controller.run_step(
                vehicle, target_wp.transform.location, target_speed))

            # Progress report every ~10%.
            done = total - len(route_q)
            pct = int(100 * done / total)
            if pct >= last_pct + 10:
                last_pct = pct
                print("[Drive] {:3d}% | speed {:4.1f} km/h | {:.0f} m to go"
                      .format(pct, VehicleController.speed_kmh(vehicle),
                              veh_loc.distance(destination)))

        # ----- PHASE 3: brake to a stop ------------------------------------
        for _ in range(int(2.0 / FIXED_DELTA)):
            vehicle.apply_control(
                carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
            world.tick()
            update_spectator(spectator, vehicle)

        if collisions:
            print("[Drive] Arrived, but {} collision(s) occurred: {}".format(
                len(collisions), set(collisions)))
        else:
            print("[Drive] Destination reached cleanly. No collisions.")

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user (Ctrl-C).")
    except Exception as exc:  # noqa: BLE001
        print("[Main] ERROR: {}".format(exc))
    finally:
        print("[Cleanup] Destroying actors and restoring settings ...")
        for actor in actor_list:
            try:
                actor.destroy()
            except Exception:
                pass
        if world is not None and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception:
                pass
        print("[Cleanup] Done.")


if __name__ == "__main__":
    main()
