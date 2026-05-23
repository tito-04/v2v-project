import unittest

from vehicle_ego.model_state import cam_model_key, model_key_for_world_candidate, upsert_model_object
from world_generator.network_metrics import NetworkMetrics
from world_generator.risk import first_risk_object, is_in_path_corridor
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
        self.assertEqual(simulation.vehicles["ego"]["x"], 30.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 4.0)
        self.assertEqual(simulation.vehicles["lead"]["x"], 90.0)
        self.assertEqual(simulation.vehicles["lead"]["y"], 4.0)
        self.assertEqual(simulation.vehicles["lead"]["heading"], 0.0)
        self.assertEqual(len(simulation.obstacles()), 3)

        reset_seen = False
        for _ in range(60):
            if simulation.tick():
                reset_seen = True
                break

        self.assertTrue(reset_seen)
        self.assertEqual(simulation.vehicles["ego"]["x"], 20.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 4.0)
        self.assertEqual(simulation.vehicles["lead"]["x"], 80.0)
        self.assertEqual(simulation.vehicles["lead"]["y"], 4.0)

    def test_intersection_ego_turns_south(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=180.0)

        simulation.tick()
        self.assertEqual(simulation.vehicles["ego"]["x"], 204.0)
        self.assertEqual(simulation.vehicles["ego"]["y"], 4.0)
        self.assertEqual(simulation.vehicles["ego"]["heading"], 270.0)
        self.assertEqual(simulation.route_indexes["ego"], 1)

        simulation.tick()
        self.assertEqual(simulation.vehicles["ego"]["y"], -176.0)
        self.assertEqual(simulation.vehicles["lead"]["heading"], 270.0)

    def test_intersection_lead_waits_for_start_delay_then_moves(self) -> None:
        scenario = load_scenario("intersection-occlusion")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=2.0)

        lead = simulation.vehicles["lead"]
        self.assertEqual(lead["y"], 200.0)
        self.assertEqual(lead["speed"], 0.0)
        self.assertEqual(lead["status"], "waiting")

        simulation.tick()
        self.assertEqual(simulation.vehicles["lead"]["y"], 200.0)
        self.assertEqual(simulation.vehicles["lead"]["status"], "waiting")

        simulation.tick()
        self.assertEqual(simulation.vehicles["lead"]["y"], 200.0)
        self.assertEqual(simulation.vehicles["lead"]["status"], "waiting")

        simulation.tick()
        self.assertEqual(simulation.vehicles["lead"]["y"], 198.0)
        self.assertEqual(simulation.vehicles["lead"]["status"], "moving")

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

    def test_vehicle_control_brakes_then_resumes_after_ttl(self) -> None:
        scenario = load_scenario("straight-obstacles")
        simulation = WorldSimulation(scenario, tick_seconds=1.0, step_meters=10.0)

        simulation.apply_control("lead", "stop", reason="test-risk", risk_object_id="obstacle_1", ttl_seconds=1.5)
        simulation.tick()
        self.assertEqual(simulation.vehicles["lead"]["x"], 80.0)
        self.assertEqual(simulation.vehicles["lead"]["speed"], 0.0)
        self.assertEqual(simulation.vehicles["lead"]["status"], "stopped")

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


if __name__ == "__main__":
    unittest.main()
