import unittest

from route_plan_planner import ClusterMemory


class ClusterMemoryTest(unittest.TestCase):
    def test_pop_uses_latest_density_and_only_pops_each_cluster_once(self):
        memory = ClusterMemory()
        first, second = memory.remember([(2, 0), (5, 0)])
        self.assertNotEqual(first.cluster_id, second.cluster_id)
        repeated, added = memory.remember([(5.05, 0), (5, .5)])
        self.assertEqual(repeated.id, second.id)
        self.assertEqual(added.cluster_id, second.cluster_id)

        selected = memory.next_cluster((0, 0, 0))
        self.assertEqual((selected.cluster_id, selected.pending_count), (second.cluster_id, 2))
        self.assertEqual({hint.id for hint in selected.hints}, {repeated.id, added.id})
        memory.remember([(5.3, 0)])
        self.assertEqual(memory.next_cluster((0, 0, 0)).cluster_id, first.cluster_id)
        self.assertFalse(memory.is_next_cluster())
        self.assertIsNone(memory.next_cluster((0, 0, 0)))

        new_point, = memory.remember([(8, 0)])
        self.assertEqual(memory.next_cluster((5, 0, 0)).cluster_id, new_point.cluster_id)

    def test_seen_again_after_missed_observation_becomes_pending(self):
        memory = ClusterMemory()
        hint, = memory.remember([(2, 0)], frame_index=2)
        memory.mark_attempted(hint.id, missed=True)
        self.assertFalse(memory.is_next_cluster())
        self.assertFalse(memory.has_unresolved_hints())
        seen, = memory.remember([(2, .1)], frame_index=3)
        self.assertEqual(seen.id, hint.id)
        self.assertTrue(memory.is_next_cluster())
        self.assertTrue(memory.has_unresolved_hints())
