"""
This is a barebone implementation, mainly for testing:
- It uses airplanes only, because hubs in these scenarios align with airports.
- Picked airplanes first because they go to hub to hub (direct)
- airplanes ignore terrain penalties (land and waters constraint), only constraint is the ground_stop_flights constraint
Features:
- It groups boxes by exact origin/destination lane.
- It spawns planes only when there is unassigned work at a hub.
- Each idle plane unloads, reloads, and immediately flies the next batch.
"""

from simulator import VehicleType, haversine_distance_meters

PROXIMITY_M = 50.0
PORT_SHUTTLE_MAX_M = 60000.0
SHIP_MIN_DISTANCE_M = 1500000.0
SHIP_MIN_BATCH = 40
AIR_DIRECT_DISTANCE_M = 7000000.0
TRAIN_MIN_BATCH = 20

_INFRA_CACHE = None


#returns true when two gps points are close enough to count as the same place
def _near(a, b, threshold=PROXIMITY_M):
    return haversine_distance_meters(a, b) <= threshold


#normalizes dict locations and tuple locations into one format
def _as_point(location):
    if isinstance(location, dict):
        return (location["lat"], location["lon"])
    return tuple(location)


#reads vehicle capacity from simulator enum instead of hard-coding it
def _vehicle_capacity(vehicle_type):
    return vehicle_type.value.capacity


#caches hub and port coordinates because infrastructure does not change
def _get_infrastructure(sim_state):
    global _INFRA_CACHE
    if _INFRA_CACHE is None:
        _INFRA_CACHE = {
            "hubs": tuple(
                _as_point(hub["location"])
                for hub in sim_state.get_shipping_hub_details()
            ),
            "ports": tuple(
                _as_point(port["location"])
                for port in sim_state.get_ocean_port_details()
            ),
        }
    return _INFRA_CACHE


#returns the matching known location when the current point is already there
def _match_location(location, candidates):
    for candidate in candidates:
        if _near(location, candidate):
            return candidate
    return None


#finds the nearest port and can optionally require shuttle distance
def _nearest_port(location, ports, max_distance=None):
    best_port = None
    best_distance = None
    for port in ports:
        distance = haversine_distance_meters(location, port)
        if max_distance is not None and distance > max_distance:
            continue
        if best_distance is None or distance < best_distance:
            best_port = port
            best_distance = distance
    return best_port


#finds idle vehicles of one type already waiting at the pickup location
def _idle_vehicles_at(vehicles, location, vehicle_type_name):
    result = []
    for vid, vehicle in vehicles.items():
        if vehicle["vehicle_type"] != vehicle_type_name:
            continue
        if vehicle["destination"] is not None:
            continue
        if _near(vehicle["location"], location):
            result.append((vid, vehicle))
    return result


#finds vehicles already traveling toward a pickup location
def _vehicles_heading_to(vehicles, location, vehicle_type_name):
    result = []
    for vid, vehicle in vehicles.items():
        if vehicle["vehicle_type"] != vehicle_type_name:
            continue
        if vehicle["destination"] is None:
            continue
        if _near(vehicle["destination"], location):
            result.append((vid, vehicle))
    return result


#unloads any idle vehicle that has reached the destination for some onboard boxes
def _deliver_idle_cargo(sim_state, vehicles, boxes):
    changed = False
    for vid, vehicle in vehicles.items():
        if vehicle["destination"] is not None or not vehicle["cargo"]:
            continue
        here = vehicle["location"]
        deliverable = [
            bid for bid in vehicle["cargo"] if _near(here, boxes[bid]["destination"])
        ]
        if deliverable:
            sim_state.unload_vehicle(vid, deliverable)
            changed = True
    if changed:
        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
    return vehicles, boxes


#groups all unassigned boxes by current location and final destination
def _build_lane_queues(boxes):
    lanes = {}
    for bid, box in boxes.items():
        if box["delivered"] or box["vehicle_id"] is not None:
            continue
        key = (box["location"], box["destination"])
        lanes.setdefault(key, []).append(bid)
    return lanes


#checks if a lane is a good candidate for truck -> ship -> truck
def _should_use_ship(origin, final_destination, batch_size, ports):
    if batch_size < SHIP_MIN_BATCH:
        return False
    if haversine_distance_meters(origin, final_destination) < SHIP_MIN_DISTANCE_M:
        return False
    origin_port = _nearest_port(origin, ports, PORT_SHUTTLE_MAX_M)
    destination_port = _nearest_port(final_destination, ports, PORT_SHUTTLE_MAX_M)
    if origin_port is None or destination_port is None:
        return False
    if _near(origin_port, destination_port):
        return False
    return True


#decides the next leg for one lane based on where the boxes are currently sitting
def _plan_lane(origin, final_destination, lane_box_ids, infrastructure):
    ports = infrastructure["ports"]
    hubs = infrastructure["hubs"]
    batch_size = len(lane_box_ids)
    origin_port = _match_location(origin, ports)
    origin_hub = _match_location(origin, hubs)
    destination_port = _nearest_port(final_destination, ports, PORT_SHUTTLE_MAX_M)

    #boxes sitting at a port should either board a ship for the destination port
    #or take a truck for the last mile trip from port to final hub
    if origin_port is not None:
        if (
            destination_port is not None
            and not _near(origin_port, destination_port)
            and batch_size >= SHIP_MIN_BATCH
        ):
            return {
                "vehicle_type": VehicleType.CargoShip,
                "spawn_location": origin_port,
                "target": destination_port,
                "mode": "ship_port_to_port",
            }
        return {
            "vehicle_type": VehicleType.SemiTruck,
            "spawn_location": None,
            "target": final_destination,
            "mode": "truck_port_to_hub",
        }

    #boxes starting at a hub can first be trucked to the nearest usable port
    #when the lane is large and long enough to justify a ship middle leg
    if origin_hub is not None and _should_use_ship(
        origin, final_destination, batch_size, ports
    ):
        return {
            "vehicle_type": VehicleType.SemiTruck,
            "spawn_location": origin,
            "target": _nearest_port(origin, ports, PORT_SHUTTLE_MAX_M),
            "mode": "truck_hub_to_port",
        }

    #otherwise keep the lane simple
    #trains for bigger lanes, trucks for smaller lanes, airplanes only as rare fallback
    if batch_size >= TRAIN_MIN_BATCH:
        return {
            "vehicle_type": VehicleType.Train,
            "spawn_location": origin,
            "target": final_destination,
            "mode": "train_direct",
        }

    if haversine_distance_meters(origin, final_destination) >= AIR_DIRECT_DISTANCE_M:
        return {
            "vehicle_type": VehicleType.Airplane,
            "spawn_location": origin,
            "target": final_destination,
            "mode": "air_direct",
        }
    return {
        "vehicle_type": VehicleType.SemiTruck,
        "spawn_location": origin,
        "target": final_destination,
        "mode": "truck_direct",
    }


#loads one vehicle with as many lane boxes as fit then sends it to the next stop
def _load_and_dispatch(sim_state, vid, vehicle, lane_box_ids, target):
    vehicle_type = VehicleType[vehicle["vehicle_type"]]
    remaining_capacity = _vehicle_capacity(vehicle_type) - len(vehicle["cargo"])
    if remaining_capacity <= 0:
        return False

    load_ids = lane_box_ids[:remaining_capacity]
    if not load_ids:
        return False

    sim_state.load_vehicle(vid, load_ids)
    refreshed_vehicle = sim_state.get_vehicles()[vid]
    if refreshed_vehicle["cargo"]:
        sim_state.move_vehicle(vid, target)
    return True


#after loading or unloading recalculate which box ids from this lane still wait
def _remaining_lane_box_ids(boxes, lane_box_ids):
    return [
        bid
        for bid in lane_box_ids
        if boxes[bid]["vehicle_id"] is None and not boxes[bid]["delivered"]
    ]


#port pickup for the final truck leg is special because trucks cannot spawn at ports
#so we spawn them at the destination hub and send them empty to the port first
def _ensure_port_trucks(sim_state, vehicles, origin, final_destination, lane_box_ids):
    needed = (
        len(lane_box_ids) + _vehicle_capacity(VehicleType.SemiTruck) - 1
    ) // _vehicle_capacity(VehicleType.SemiTruck)
    available = len(_idle_vehicles_at(vehicles, origin, "SemiTruck")) + len(
        _vehicles_heading_to(vehicles, origin, "SemiTruck")
    )

    while available < needed:
        vid = sim_state.create_vehicle(VehicleType.SemiTruck, final_destination)
        sim_state.move_vehicle(vid, origin)
        available += 1
        vehicles = sim_state.get_vehicles()
    return vehicles


#called once per simulation tick
def step(sim_state):
    infrastructure = _get_infrastructure(sim_state)
    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    #first finish any deliveries for vehicles that have already arrived
    vehicles, boxes = _deliver_idle_cargo(sim_state, vehicles, boxes)

    lanes = _build_lane_queues(boxes)
    if not lanes:
        return

    #reuse idle vehicles already parked at the current origin first
    for (origin, final_destination), lane_box_ids in list(lanes.items()):
        plan = _plan_lane(origin, final_destination, lane_box_ids, infrastructure)
        for vid, vehicle in _idle_vehicles_at(
            vehicles, origin, plan["vehicle_type"].name
        ):
            if not lane_box_ids:
                break
            if vehicle["cargo"]:
                continue
            if _load_and_dispatch(
                sim_state, vid, vehicle, lane_box_ids, plan["target"]
            ):
                vehicles = sim_state.get_vehicles()
                boxes = sim_state.get_boxes()
                lane_box_ids = _remaining_lane_box_ids(boxes, lane_box_ids)

        if lane_box_ids:
            lanes[(origin, final_destination)] = lane_box_ids
        else:
            del lanes[(origin, final_destination)]

    #spawn new vehicles for lanes that still have unassigned boxes
    for (origin, final_destination), lane_box_ids in list(lanes.items()):
        if not lane_box_ids:
            continue

        plan = _plan_lane(origin, final_destination, lane_box_ids, infrastructure)

        if plan["mode"] == "truck_port_to_hub":
            vehicles = _ensure_port_trucks(
                sim_state, vehicles, origin, final_destination, lane_box_ids
            )
            continue

        while lane_box_ids:
            vid = sim_state.create_vehicle(plan["vehicle_type"], plan["spawn_location"])
            vehicle = sim_state.get_vehicles()[vid]
            if not _load_and_dispatch(
                sim_state, vid, vehicle, lane_box_ids, plan["target"]
            ):
                break
            vehicles = sim_state.get_vehicles()
            boxes = sim_state.get_boxes()
            lane_box_ids = _remaining_lane_box_ids(boxes, lane_box_ids)
