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

#returns true when two GPS points are close enough to count as "the same place" for delivery
def _near(a, b, threshold=PROXIMITY_M):
    return haversine_distance_meters(a, b) <= threshold

#validates plane capacity
def _plane_capacity():
    return VehicleType.Airplane.value.capacity

#find all airplanes that are currently usable, given the origin
#Usable means:
#- its an Airplane
#- its idle
#- its already at the origin and can be reused
def _idle_airplanes_at(vehicles, location):
    result = []
    for vid, vehicle in vehicles.items():
        if vehicle["vehicle_type"] != "Airplane":
            continue
        if vehicle["destination"] is not None:
            continue
        if _near(vehicle["location"], location):
            result.append((vid, vehicle))
    return result


#when a plane has arrived, unload only the boxes where the destination matches
#the plane's current location; Basically prevents unloading cargo at the wrong hub
def _deliver_idle_cargo(sim_state, vehicles, boxes):
    changed = False
    for vid, vehicle in vehicles.items():
        #skip planes that are still in transit or have nothing to unload
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
        #refreshs to keep cargo and box state accurate for the rest of the tick
        vehicles = sim_state.get_vehicles()
        boxes = sim_state.get_boxes()
    return vehicles, boxes

#delivered boxes and boxes already assigned to a vehicle are skipped because they dont represent new work to plan
def _build_lane_queues(boxes):
    lanes = {}
    for bid, box in boxes.items():
        if box["delivered"] or box["vehicle_id"] is not None:
            continue
        key = (box["location"], box["destination"])
        lanes.setdefault(key, []).append(bid)
    return lanes

#loads one airplane with as many boxes as possible from a single lane and dispatches directly to the destination
def _load_and_dispatch(sim_state, vid, vehicle, boxes, lane_box_ids):
    #capacity is reduced by boxes the plane is already carrying
    remaining_capacity = _plane_capacity() - len(vehicle["cargo"])
    if remaining_capacity <= 0:
        return False

    #takes only as many boxes as capacity allows it to
    load_ids = lane_box_ids[:remaining_capacity]
    if not load_ids:
        return False

    sim_state.load_vehicle(vid, load_ids)
    refreshed_vehicle = sim_state.get_vehicles()[vid]
    if refreshed_vehicle["cargo"]:
        #all boxes in the lane share one destination, first loaded box determines where the plane should fly
        destination = boxes[refreshed_vehicle["cargo"][0]]["destination"]
        sim_state.move_vehicle(vid, destination)
    return True

#main control loop called once per simulation tick
#order:
#1. finish deliveries for planes that have landed
#2. rebuild remaining work queue
#3. reuse idle local planes first
#4. spawn new planes only for leftover work
def step(sim_state):
    #reads the simulator snapshots for this tick.
    boxes = sim_state.get_boxes()
    vehicles = sim_state.get_vehicles()

    #first, complete any deliveries for airplanes that have arrived
    vehicles, boxes = _deliver_idle_cargo(sim_state, vehicles, boxes)

    #rebuild work queues after unloading
    lanes = _build_lane_queues(boxes)
    if not lanes:
        return

    #reuse idle airplanes already sitting at the right origin first
    for (origin, _destination), lane_box_ids in list(lanes.items()):
        for vid, vehicle in _idle_airplanes_at(vehicles, origin):
            if not lane_box_ids:
                break
            #ignore reused planes that somehow still has cargo, assumes one trip is fully planned before the next
            if vehicle["cargo"]:
                continue
            if _load_and_dispatch(sim_state, vid, vehicle, boxes, lane_box_ids):
                #refresh state after loading so already-assigned boxes are not considered available for another plane in the same tick.
                vehicles = sim_state.get_vehicles()
                boxes = sim_state.get_boxes()
                lane_box_ids = [
                    bid
                    for bid in lane_box_ids
                    if boxes[bid]["vehicle_id"] is None and not boxes[bid]["delivered"]
                ]
        if lane_box_ids:
            lanes[(origin, _destination)] = lane_box_ids
        else:
            del lanes[(origin, _destination)]

    #spawns new airplanes only for remaining work that has no idle plane available.
    for (origin, _destination), lane_box_ids in lanes.items():
        while lane_box_ids:
            #non-efficient part: every time there is leftover work on a lane, we create another airplane instead of waiting
            vid = sim_state.create_vehicle(VehicleType.Airplane, origin)
            vehicle = sim_state.get_vehicles()[vid]
            _load_and_dispatch(sim_state, vid, vehicle, boxes, lane_box_ids)
            boxes = sim_state.get_boxes()
            #Remove boxes that were just loaded so the while-loop eventually terminates
            lane_box_ids = [
                bid
                for bid in lane_box_ids
                if boxes[bid]["vehicle_id"] is None and not boxes[bid]["delivered"]
            ]
