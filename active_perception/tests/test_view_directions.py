import math
import unittest

from active_perception_sdk import ObservationCue, suggest_views


class ViewDirectionTests(unittest.TestCase):
    def directions(self, bearings, fov=30, tried=()):
        result = suggest_views(
            (ObservationCue(math.radians(angle)) for angle in bearings),
            horizontal_fov=math.radians(fov),
            tried_yaw_offsets=map(math.radians, tried),
        )
        return [round(math.degrees(view.yaw_offset), 8) for view in result]

    def test_current_view_and_neighboring_cues_share_suggestions(self):
        self.assertEqual(self.directions([8, -8, 38, 55, 38, -75]), [0, 46.5, -75])

    def test_bearings_on_both_sides_of_pi_use_one_view(self):
        self.assertEqual(self.directions([172, -176]), [178])

    def test_tried_view_only_excludes_cues_in_its_field_of_view(self):
        self.assertEqual(self.directions([9, 37, 55, -60], tried=[37]), [0, 55, -60])
        self.assertEqual(self.directions([178, -179, 70], tried=[-175]), [70])

    def test_empty_or_fully_observed_returns_no_suggestions(self):
        self.assertEqual(self.directions([]), [])
        self.assertEqual(self.directions([0, 15, -15], tried=[0]), [])

    def test_wide_arc_produces_views_that_cover_every_cue(self):
        bearings = [22, 42, 62, 82, 102, 125, 166, -150, -100, -42, -22]
        headings = self.directions(bearings, fov=45)
        for bearing in bearings:
            with self.subTest(bearing=bearing):
                self.assertTrue(any(abs((bearing - yaw + 180) % 360 - 180) <= 22.5 for yaw in headings))

    def test_reported_bearing_controls_yaw_even_when_position_is_noisy(self):
        cues = [ObservationCue(math.radians(42), (3, 4)), ObservationCue(math.radians(45), (4, -3))]
        view, = suggest_views(cues, horizontal_fov=math.radians(30))
        self.assertAlmostEqual(math.degrees(view.yaw_offset), 43.5)
        self.assertEqual(view.cue_indices, (0, 1))
        self.assertEqual(view.estimated_nearest_distance_m, 5)

    def test_missing_position_does_not_prevent_a_suggestion(self):
        view, = suggest_views([ObservationCue(0.4)], horizontal_fov=0.2)
        self.assertAlmostEqual(view.yaw_offset, 0.4)
        self.assertIsNone(view.estimated_nearest_distance_m)

    def test_invalid_field_of_view_and_bearings_are_rejected(self):
        for fov in [0, -1, math.tau + 1, math.inf, math.nan]:
            with self.subTest(fov=fov), self.assertRaises(ValueError):
                suggest_views([ObservationCue(0.1)], horizontal_fov=fov)
        with self.assertRaises(ValueError):
            self.directions([math.nan])
        with self.assertRaises(ValueError):
            self.directions([20], tried=[math.inf])
        with self.assertRaises(ValueError):
            ObservationCue(0.1, (math.nan, 0))


if __name__ == "__main__":
    unittest.main()
