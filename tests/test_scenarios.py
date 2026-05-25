import unittest

from vehicle_ego.model_state import cam_model_key, match_world_object_by_position, model_key_for_world_candidate, upsert_model_object
from vehicle_lead.control_policy import lead_control_risk
from vehicle_lead.cpm_payload import build_cpm_payload
from world_generator.network_metrics import NetworkMetrics
from world_generator.risk import first_risk_object, is_in_fov, is_in_path_corridor
from world_generator.scenarios import available_scenarios, load_scenario, public_metadata
from world_generator.simulation import WorldSimulation


class ScenarioLoaderTests(unittest.TestCase):
    def test_lists_initial_scenarios(self) -> None:
        self.assertIn("intersection-occlusion", available_scenarios())
        self.assertIn("straight-obstacles", available_scenarios())

    def test_public_metadata_exposes_layout(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        metadata = public_metadata(scenario)

        self.assertEqual(metadata["name"], "intersection-occlusion")
        self.assertGreaterEqual(len(metadata["layout"]["roads"]), 2)
        self.assertEqual(metadata["layout"]["occluders"][0]["type"], "rect")
        self.assertEqual(metadata["layout"]["occluders"][0]["x1"], 176)
        self.assertEqual(metadata["layout"]["occluders"][0]["x2"], 192)
        self.assertEqual(metadata["layout"]["occluders"][0]["y2"], -8)


class WorldSimulationTests(unittest.TestCase):
    def test_straight_obstacles_moves_both_vehicles_east_and_resets(self) -> None:
        scenario = load_scenario("straight-obstacles")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=10.0)

        simulation.tick()
        self.assertEqual(simulation.vehicles["ego"]["x"], 32.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 12.0)
        self.assertEqual(simulation.vehicles["lead"]["x"], 94.0)
        self.assertEqual(simulation.vehicles["lead"]["y"], 12.0)
        self.assertEqual(simulation.vehicles["lead"]["heading"], 0.0)
        self.assertEqual(len(simulation.obstacles()), 3)
        self.assertTrue(all(not obstacle["blocks_vehicle_path"] for obstacle in simulation.obstacles()))

        reset_seen = False
        for _ in range(60):
            if simulation.tick():
                reset_seen = True
                break

        self.assertTrue(reset_seen)
        self.assertEqual(simulation.vehicles["ego"]["x"], 20.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 12.0)
        self.assertEqual(simulation.vehicles["lead"]["x"], 80.0)
        self.assertEqual(simulation.vehicles["lead"]["y"], 12.0)

    def test_intersection_ego_can_turn_south_without_cpm_risk(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=2.0)

        self.assertEqual(simulation.vehicles["ego"]["x"], 70.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 4.0)
        self.assertEqual(simulation.vehicles["ego"]["heading"], 0.0)

        for _ in range(10):
            simulation.tick()

        self.assertEqual(simulation.vehicles["ego"]["x"], 204.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 4.0)
        self.assertEqual(simulation.vehicles["ego"]["heading"], 270.0)
        self.assertEqual(simulation.route_indexes["ego"], 1)

        simulation.tick()
        self.assertEqual(simulation.vehicles["ego"]["y"], -9.0)
        self.assertEqual(simulation.vehicles["lead"]["heading"], 270.0)

    def test_intersection_lead_moves_immediately_and_detects_before_ego_entry(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        occluders = scenario["layout"]["occluders"]
        simulation = WorldSimulation(scenario, tick_seconds=0.1, step_meters=1.0)

        lead = simulation.vehicles["lead"]
        self.assertEqual(lead["y"], 200.0)
        self.assertEqual(lead["status"], "moving")

        detected = False
        for _ in range(120):
            simulation.tick()
            lead = simulation.vehicles["lead"]
            pedestrian = simulation.obstacles()[0]
            detected = is_in_fov(
                lead["x"], lead["y"], lead["heading"],
                pedestrian["x"], pedestrian["y"],
                80.0, 60.0, occluders,
            )
            if detected:
                break

        self.assertTrue(detected)
        self.assertEqual(simulation.vehicles["lead"]["status"], "moving")
        self.assertLess(simulation.vehicles["ego"]["x"], 184.0)

    def test_intersection_pedestrian_waits_for_ego_stop_trigger(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=2.0)

        pedestrian = simulation.obstacles()[0]
        self.assertEqual(pedestrian["id"], "pedestrian-1")
        self.assertEqual(pedestrian["kind"], "pedestrian")
        self.assertTrue(pedestrian["blocks_vehicle_path"])
        self.assertEqual(pedestrian["x"], 198.0)
        self.assertEqual(pedestrian["status"], "waiting")

        simulation.tick()
        pedestrian = simulation.obstacles()[0]
        self.assertEqual(pedestrian["x"], 198.0)
        self.assertEqual(pedestrian["status"], "waiting")

        simulation.apply_control("ego", "stop", reason="ego-model-risk", risk_object_id="pedestrian-1", ttl_seconds=5.0)
        for _ in range(20):
            simulation.tick()
            if simulation.vehicles["ego"]["status"] == "stopped":
                break
        simulation.tick()
        pedestrian = simulation.obstacles()[0]
        self.assertGreater(pedestrian["x"], 198.0)
        self.assertEqual(pedestrian["status"], "crossing")

    def test_intersection_pedestrian_stops_done_after_crossing_once(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=2.0)
        simulation.apply_control("ego", "stop", reason="ego-model-risk", risk_object_id="pedestrian-1", ttl_seconds=30.0)

        for _ in range(20):
            simulation.tick()

        pedestrian = simulation.obstacles()[0]
        self.assertEqual(pedestrian["x"], 218.0)
        self.assertEqual(pedestrian["speed"], 0.0)
        self.assertEqual(pedestrian["status"], "done")

    def test_intersection_stop_at_keeps_ego_before_turn_for_pedestrian_cpm(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=0.1, step_meters=1.0)

        for _ in range(85):
            simulation.tick()

        self.assertLess(simulation.vehicles["ego"]["x"], 184.0)
        simulation.apply_control(
            "ego",
            "stop_at",
            reason="ego-model-risk",
            risk_object_id="pedestrian-1",
            ttl_seconds=20.0,
            stop_axis="x",
            stop_value=184.0,
            stop_direction=1,
            stop_x=184.0,
            stop_y=4.0,
            stop_heading=0.0,
            stop_route_index=0,
        )

        for _ in range(80):
            simulation.tick()
            if simulation.vehicles["ego"]["status"] == "stopped":
                break

        ego = simulation.vehicles["ego"]
        pedestrian = simulation.obstacles()[0]
        self.assertEqual(ego["x"], 184.0)
        self.assertEqual(ego["y"], 4.0)
        self.assertEqual(ego["heading"], 0.0)
        self.assertEqual(simulation.route_indexes["ego"], 0)
        self.assertEqual(ego["status"], "stopped")
        self.assertEqual(ego["risk_object_id"], "pedestrian-1")
        self.assertEqual(pedestrian["status"], "crossing")

    def test_intersection_late_stop_at_parks_ego_back_before_turn(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=0.1, step_meters=1.0)

        while simulation.route_indexes["ego"] == 0:
            simulation.tick()

        self.assertEqual(simulation.route_indexes["ego"], 1)
        simulation.apply_control(
            "ego",
            "stop_at",
            reason="ego-model-risk",
            risk_object_id="pedestrian-1",
            ttl_seconds=20.0,
            stop_axis="x",
            stop_value=184.0,
            stop_direction=1,
            stop_x=184.0,
            stop_y=4.0,
            stop_heading=0.0,
            stop_route_index=0,
        )
        simulation.tick()

        ego = simulation.vehicles["ego"]
        pedestrian = simulation.obstacles()[0]
        self.assertEqual(ego["x"], 184.0)
        self.assertEqual(ego["y"], 4.0)
        self.assertEqual(ego["heading"], 0.0)
        self.assertEqual(simulation.route_indexes["ego"], 0)
        self.assertEqual(ego["status"], "stopped")
        self.assertEqual(ego["risk_object_id"], "pedestrian-1")
        self.assertEqual(pedestrian["status"], "crossing")

    def test_intersection_ego_resumes_turn_after_stop_line_control_clears(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=0.1, step_meters=1.0)

        simulation.apply_control(
            "ego",
            "stop_at",
            reason="ego-model-risk",
            risk_object_id="pedestrian-1",
            ttl_seconds=20.0,
            stop_axis="x",
            stop_value=184.0,
            stop_direction=1,
            stop_x=184.0,
            stop_y=4.0,
            stop_heading=0.0,
            stop_route_index=0,
        )
        for _ in range(120):
            simulation.tick()
            if simulation.vehicles["ego"]["status"] == "stopped":
                break

        self.assertEqual(simulation.vehicles["ego"]["x"], 184.0)
        self.assertEqual(simulation.route_indexes["ego"], 0)

        simulation.apply_control("ego", "resume")
        for _ in range(80):
            simulation.tick()
            if simulation.route_indexes["ego"] == 1:
                break

        ego = simulation.vehicles["ego"]
        self.assertEqual(simulation.route_indexes["ego"], 1)
        self.assertEqual(ego["x"], 204.0)
        self.assertEqual(ego["y"], 4.0)
        self.assertEqual(ego["heading"], 270.0)

    def test_intersection_lead_resumes_at_ego_speed_after_pedestrian_stop(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=0.1, step_meters=1.0)

        self.assertEqual(simulation.vehicles["ego"]["base_speed"], 13.0)
        self.assertEqual(simulation.vehicles["lead"]["base_speed"], 16.0)

        simulation.apply_control("lead", "stop", reason="path-risk", risk_object_id="pedestrian-1", ttl_seconds=20.0)
        for _ in range(40):
            simulation.tick()
            if simulation.vehicles["lead"]["status"] == "stopped":
                break

        self.assertEqual(simulation.vehicles["lead"]["status"], "stopped")
        simulation.apply_control("lead", "resume")

        self.assertEqual(simulation.vehicles["lead"]["base_speed"], simulation.vehicles["ego"]["base_speed"])
        self.assertEqual(simulation.vehicles["lead"]["target_speed"], simulation.vehicles["ego"]["base_speed"])

    def test_vehicle_control_brakes_then_resumes_after_ttl(self) -> None:
        scenario = load_scenario("straight-obstacles")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=10.0)

        simulation.apply_control("lead", "stop", reason="test-risk", risk_object_id="obstacle_1", ttl_seconds=1.5)
        simulation.tick()
        self.assertEqual(simulation.vehicles["lead"]["x"], 82.0)
        self.assertEqual(simulation.vehicles["lead"]["speed"], 2.0)
        self.assertEqual(simulation.vehicles["lead"]["status"], "braking")

        simulation.tick()
        self.assertEqual(simulation.vehicles["lead"]["x"], 90.0)
        self.assertEqual(simulation.vehicles["lead"]["status"], "moving")


class RiskGeometryTests(unittest.TestCase):
    def test_path_corridor_detects_only_objects_ahead(self) -> None:
        self.assertTrue(is_in_path_corridor(204.0, 20.0, 270.0, 204.0, -14.0, lookahead_m=70.0, half_width_m=7.0))
        self.assertFalse(is_in_path_corridor(204.0, 20.0, 270.0, 230.0, -14.0, lookahead_m=70.0, half_width_m=7.0))
        self.assertFalse(is_in_path_corridor(204.0, 20.0, 270.0, 204.0, 90.0, lookahead_m=70.0, half_width_m=7.0))

    def test_first_risk_object_ignores_non_blocking_objects(self) -> None:
        vehicle = {"x": 204.0, "y": 20.0, "heading": 270.0}
        objects = [
            {"id": "ghost", "x": 204.0, "y": 0.0, "blocks_vehicle_path": False},
            {"id": "ped", "x": 204.0, "y": -10.0, "blocks_vehicle_path": True},
        ]

        risk = first_risk_object(vehicle, objects, lookahead_m=70.0, half_width_m=7.0)
        self.assertIsNotNone(risk)
        self.assertEqual(risk["id"], "ped")


class NetworkMetricsTests(unittest.TestCase):
    def test_cpm_payload_includes_non_blocking_obstacle_metadata(self) -> None:
        payload = build_cpm_payload(
            80.0,
            12.0,
            [{
                "object_id": "1",
                "kind": "obstacle",
                "x": 100.0,
                "y": -12.0,
                "blocks_vehicle_path": False,
            }],
            base_lat=40.628300,
            base_lon=-8.654400,
            fov_range_m=80.0,
            fov_half_angle_deg=60.0,
        )

        objects = payload["cpmContainers"][1]["containerData"]["perceivedObjects"]
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]["objectPublicId"], "1")
        self.assertEqual(objects[0]["kind"], "obstacle")
        self.assertFalse(objects[0]["blocksVehiclePath"])

    def test_tracks_delay_and_loss(self) -> None:
        metrics = NetworkMetrics(loss_timeout_seconds=2.0)

        metrics.record_tx("cam", sequence=1, generated_at=0.0, now=0.0)
        metrics.record_rx("cam", now=0.5)
        metrics.record_tx("cam", sequence=2, generated_at=1.0, now=1.0)
        metrics.sweep(now=4.0)

        snapshot = metrics.snapshot()
        self.assertEqual(snapshot["cam"]["sent"], 2)
        self.assertEqual(snapshot["cam"]["received"], 1)
        self.assertEqual(snapshot["cam"]["lost"], 1)
        self.assertEqual(snapshot["cam"]["loss_percent"], 50.0)
        self.assertEqual(snapshot["cam"]["last_delay_sec"], 0.5)


class LeadControlPolicyTests(unittest.TestCase):
    def test_non_blocking_objects_can_be_perceived_without_lead_control_risk(self) -> None:
        perceived = [{
            "object_id": "1",
            "x": 100.0,
            "y": -12.0,
            "blocks_vehicle_path": False,
        }]
        risk = first_risk_object({"x": 80.0, "y": 12.0, "heading": 0.0}, perceived)

        control_risk, held = lead_control_risk(risk, {"obstacle_1": {"id": "1"}}, None)

        self.assertIsNone(control_risk)
        self.assertIsNone(held)

    def test_lead_holds_pedestrian_risk_until_crossing_done(self) -> None:
        world_objects = {
            "obstacle_pedestrian-1": {"id": "pedestrian-1", "status": "crossing"}
        }
        risk = {"object_id": "obstacle_pedestrian-1", "blocks_vehicle_path": True}

        control_risk, held = lead_control_risk(risk, world_objects, None)
        self.assertEqual(control_risk["object_id"], "pedestrian-1")
        self.assertEqual(held, "pedestrian-1")

        control_risk, held = lead_control_risk(None, world_objects, held)
        self.assertEqual(control_risk["object_id"], "pedestrian-1")
        self.assertEqual(held, "pedestrian-1")

        world_objects["obstacle_pedestrian-1"]["status"] = "done"
        control_risk, held = lead_control_risk(None, world_objects, held)
        self.assertIsNone(control_risk)
        self.assertIsNone(held)


class EgoModelMergeTests(unittest.TestCase):
    def test_direct_and_cam_lead_share_one_model_key(self) -> None:
        objects: dict[str, dict[str, object]] = {}
        direct_lead = {
            "id": "lead",
            "kind": "vehicle",
            "x": 204.0,
            "y": 20.0,
            "source": "direct",
            "updated_at": 10.0,
        }
        direct_key = model_key_for_world_candidate("vehicle_lead", direct_lead)
        upsert_model_object(objects, direct_key, direct_lead, source_priority=30)

        cam_key = cam_model_key(objects, 205.0, 21.0, station_id=101)
        upsert_model_object(
            objects,
            cam_key,
            {"id": cam_key, "kind": "vehicle", "x": 205.0, "y": 21.0, "source": "cam", "updated_at": 11.0},
            source_priority=10,
        )

        self.assertEqual(cam_key, "lead")
        self.assertEqual(list(objects), ["lead"])
        self.assertEqual(objects["lead"]["source"], "direct")
        self.assertIn("cam", objects["lead"]["secondary_sources"])

    def test_direct_obstacle_replaces_cpm_without_duplicate(self) -> None:
        objects: dict[str, dict[str, object]] = {}
        world_obstacle = {
            "id": "1",
            "kind": "obstacle",
            "x": 100.0,
            "y": -12.0,
            "blocks_vehicle_path": False,
        }
        matched = match_world_object_by_position({"1": world_obstacle}, 100.4, -12.2, max_distance_m=5.0)
        self.assertIsNotNone(matched)

        key, matched_obstacle = matched
        cpm_item = dict(matched_obstacle)
        cpm_item.update({
            "source": "cpm",
            "observed_via": "v2v_cpm",
            "detected_by": 101,
            "updated_at": 10.0,
            "stale": False,
        })
        upsert_model_object(objects, key, cpm_item, source_priority=20)

        direct_key = model_key_for_world_candidate("object_1", world_obstacle)
        direct_item = dict(world_obstacle)
        direct_item.update({
            "source": "direct",
            "observed_via": "ego_sensor",
            "updated_at": 11.0,
            "stale": False,
        })
        upsert_model_object(objects, direct_key, direct_item, source_priority=30)

        self.assertEqual(direct_key, "1")
        self.assertEqual(list(objects), ["1"])
        self.assertEqual(objects["1"]["source"], "direct")
        self.assertFalse(objects["1"]["blocks_vehicle_path"])
        self.assertIn("cpm", objects["1"]["secondary_sources"])

    def test_cpm_position_matches_world_pedestrian_key(self) -> None:
        world_objects = {
            "pedestrian-1": {
                "id": "pedestrian-1",
                "kind": "pedestrian",
                "x": 198.0,
                "y": -14.0,
                "blocks_vehicle_path": True,
                "status": "waiting",
            }
        }

        matched = match_world_object_by_position(world_objects, 198.8, -13.4, max_distance_m=5.0)

        self.assertIsNotNone(matched)
        key, obj = matched
        self.assertEqual(key, "pedestrian-1")
        self.assertEqual(obj["id"], "pedestrian-1")


if __name__ == "__main__":
    unittest.main()
