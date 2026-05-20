import unittest

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


if __name__ == "__main__":
    unittest.main()
