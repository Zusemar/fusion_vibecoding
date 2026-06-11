import ast
import math
from pathlib import Path
import unittest


GENERATOR = (
    Path(__file__).parents[1]
    / 'Lab3_V4_Generator'
    / 'Lab3_V4_Generator.py'
)


class GeneratorSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(GENERATOR.read_text(encoding='utf-8'))
        cls.constants = {
            node.targets[0].id: ast.literal_eval(node.value)
            for node in cls.tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and isinstance(node.value, (ast.Tuple, ast.Constant))
        }

    def test_make_design_does_not_rename_root_component(self):
        make_design = next(
            node
            for node in self.tree.body
            if isinstance(node, ast.FunctionDef) and node.name == 'make_design'
        )

        root_name_assignments = [
            node
            for node in ast.walk(make_design)
            if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign))
            and any(
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == 'root'
                and target.attr == 'name'
                for target in self._targets(node)
            )
        ]

        self.assertEqual(root_name_assignments, [])

    def test_basis_transform_derives_right_handed_z_axis(self):
        basis_transform = next(
            node
            for node in self.tree.body
            if isinstance(node, ast.FunctionDef) and node.name == 'basis_transform'
        )

        self.assertEqual(
            [argument.arg for argument in basis_transform.args.args],
            ['origin', 'x_axis', 'y_axis'],
        )
        self.assertTrue(
            any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == 'cross_product'
                for node in ast.walk(basis_transform)
            )
        )

    def test_literal_basis_transform_axes_are_orthonormal(self):
        calls = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'basis_transform'
        ]

        self.assertGreater(len(calls), 0)
        for call in calls:
            self.assertEqual(len(call.args), 3)
            x_axis = ast.literal_eval(call.args[1])
            y_axis = ast.literal_eval(call.args[2])
            self.assertAlmostEqual(sum(value * value for value in x_axis), 1.0)
            self.assertAlmostEqual(sum(value * value for value in y_axis), 1.0)
            self.assertAlmostEqual(
                sum(left * right for left, right in zip(x_axis, y_axis)),
                0.0,
            )

    def test_task_2_uses_realistic_fastener_layout(self):
        bracket_positions = self.constants['BRACKET_X_POSITIONS']
        anchor_x_offsets = self.constants['WALL_ANCHOR_X_OFFSETS']
        anchor_z_positions = self.constants['WALL_ANCHOR_Z_POSITIONS']
        shelf_screw_positions = self.constants['SHELF_SCREW_Y_POSITIONS']

        self.assertEqual(len(bracket_positions), 3)
        self.assertEqual(len(anchor_x_offsets) * len(anchor_z_positions), 4)
        self.assertEqual(len(shelf_screw_positions), 2)

    def test_task_2_contains_twin_ribs_and_visible_fastener_hardware(self):
        source = GENERATOR.read_text(encoding='utf-8')

        self.assertIn("enumerate((-50, 42), start=1)", source)
        self.assertIn("'SIDE RIB {rib_index}", source)
        self.assertIn("'ANCHOR {index}", source)
        self.assertIn("'SHELF SCREW {index}", source)
        self.assertIn('add_washer(', source)

    def test_task_3_reference_spur_gears_do_not_intersect(self):
        teeth = self.constants['TASK_34_GEAR_TEETH']
        module = self.constants['TASK_34_GEAR_MODULE']
        center_distance = (
            module * teeth
            + self.constants['TASK_34_CENTER_CLEARANCE']
        )
        first = self._reference_spur_gear_outline(
            teeth,
            module,
            self.constants['TASK_34_PRESSURE_ANGLE_DEG'],
            self.constants['TASK_34_HOLE_DIAMETER_MM'],
            self.constants['TASK_34_ROOT_FILLET_MM'],
            0,
            0,
        )
        second = self._reference_spur_gear_outline(
            teeth,
            module,
            self.constants['TASK_34_PRESSURE_ANGLE_DEG'],
            self.constants['TASK_34_HOLE_DIAMETER_MM'],
            self.constants['TASK_34_ROOT_FILLET_MM'],
            center_distance,
            45,
        )

        self.assertEqual(self.constants['TASK_34_PRESSURE_ANGLE_DEG'], 20)
        self.assertEqual(self.constants['TASK_34_THICKNESS_MM'], 10)
        self.assertEqual(self.constants['TASK_34_HOLE_DIAMETER_MM'], 5)
        self.assertFalse(self._polygon_self_intersects(first))
        self.assertFalse(self._polygon_self_intersects(second))
        self.assertFalse(self._polygons_intersect(first, second))

    def test_task_4_uses_reference_spur_optimization_candidate(self):
        source = GENERATOR.read_text(encoding='utf-8')

        self.assertIn(
            "'OPTIMIZATION CANDIDATE — 4 REFERENCE SPUR teeth'",
            source,
        )
        self.assertIn('add_reference_spur_gear(', source)

    def test_task_6_simplified_gears_have_clearance(self):
        teeth_1, teeth_2 = self.constants['TASK_6_GEAR_TEETH']
        module = self.constants['TASK_6_GEAR_MODULE']
        clearance = self.constants['TASK_6_CENTER_CLEARANCE']
        center_distance = module * (teeth_1 + teeth_2) / 2 + clearance
        outer_radius_sum = (
            module * teeth_1 / 2
            + module
            + module * teeth_2 / 2
            + module
        )

        self.assertEqual(center_distance, 70)
        self.assertGreater(center_distance, module * (teeth_1 + teeth_2) / 2)
        self.assertLess(center_distance, outer_radius_sum)

    def test_task_6_gear_outlines_do_not_intersect_during_one_driver_turn(self):
        teeth_1, teeth_2 = self.constants['TASK_6_GEAR_TEETH']
        module = self.constants['TASK_6_GEAR_MODULE']
        center_distance = (
            module * (teeth_1 + teeth_2) / 2
            + self.constants['TASK_6_CENTER_CLEARANCE']
        )

        for step in range(129):
            driver_angle = 360 * step / 128
            driven_angle = 22.5 - driver_angle / 2
            driver = self._gear_outline(teeth_1, module, 8, 0, driver_angle)
            driven = self._gear_outline(
                teeth_2,
                module,
                10,
                center_distance,
                driven_angle,
            )
            self.assertFalse(self._polygons_intersect(driver, driven))

    @staticmethod
    def _gear_outline(teeth, module, bore, center_x, angle_degrees):
        pitch_radius = module * teeth / 2
        outer_radius = pitch_radius + module
        root_radius = max(bore * 0.8, pitch_radius - 1.25 * module)
        angular_pitch = 2 * math.pi / teeth
        rotation = math.radians(angle_degrees)
        outline = []

        for tooth in range(teeth):
            center_angle = tooth * angular_pitch + rotation
            for fraction, radius in (
                (-0.50, root_radius),
                (-0.30, root_radius),
                (-0.19, outer_radius),
                (0.19, outer_radius),
                (0.30, root_radius),
            ):
                angle = center_angle + fraction * angular_pitch
                outline.append(
                    (
                        center_x + radius * math.cos(angle),
                        radius * math.sin(angle),
                    )
                )

        return outline

    @staticmethod
    def _reference_spur_gear_outline(
        teeth,
        module,
        pressure_angle_deg,
        bore,
        root_fillet,
        center_x,
        angle_degrees,
    ):
        pitch_radius = module * teeth / 2
        base_radius = pitch_radius * math.cos(math.radians(pressure_angle_deg))
        outer_radius = pitch_radius + module
        root_radius = max(
            pitch_radius - 1.25 * module,
            bore / 2 + 2 * root_fillet,
        )
        angular_pitch = 2 * math.pi / teeth
        pitch_half_tooth_angle = math.pi / (2 * teeth)
        pitch_involute = math.tan(math.radians(pressure_angle_deg)) - math.radians(
            pressure_angle_deg
        )
        rotation = math.radians(angle_degrees)

        def involute_angle(angle):
            return math.tan(angle) - angle

        def flank_angle(radius):
            return (
                pitch_half_tooth_angle
                + pitch_involute
                - involute_angle(math.acos(base_radius / radius))
            )

        base_angle = flank_angle(base_radius)
        outer_angle = flank_angle(outer_radius)
        outline = []

        for tooth in range(teeth):
            center_angle = rotation + tooth * angular_pitch
            for index in range(7):
                ratio = index / 6
                smooth_ratio = ratio * ratio * (3 - 2 * ratio)
                angle = (
                    center_angle
                    - angular_pitch / 2
                    + (angular_pitch / 2 - base_angle) * smooth_ratio
                )
                radius = root_radius + (base_radius - root_radius) * smooth_ratio
                outline.append(
                    (center_x + radius * math.cos(angle), radius * math.sin(angle))
                )
            for index in range(1, 10):
                ratio = index / 9
                radius = base_radius + (outer_radius - base_radius) * ratio
                angle = center_angle - flank_angle(radius)
                outline.append(
                    (center_x + radius * math.cos(angle), radius * math.sin(angle))
                )
            outline.append(
                (
                    center_x + outer_radius * math.cos(center_angle + outer_angle),
                    outer_radius * math.sin(center_angle + outer_angle),
                )
            )
            for index in range(1, 10):
                ratio = index / 9
                radius = outer_radius + (base_radius - outer_radius) * ratio
                angle = center_angle + flank_angle(radius)
                outline.append(
                    (center_x + radius * math.cos(angle), radius * math.sin(angle))
                )
            for index in range(1, 6):
                ratio = index / 6
                smooth_ratio = ratio * ratio * (3 - 2 * ratio)
                angle = (
                    center_angle
                    + base_angle
                    + (angular_pitch / 2 - base_angle) * smooth_ratio
                )
                radius = base_radius + (root_radius - base_radius) * smooth_ratio
                outline.append(
                    (center_x + radius * math.cos(angle), radius * math.sin(angle))
                )

        return outline

    @classmethod
    def _polygon_self_intersects(cls, polygon):
        edge_count = len(polygon)
        for first_index in range(edge_count):
            first_next = (first_index + 1) % edge_count
            for second_index in range(first_index + 1, edge_count):
                second_next = (second_index + 1) % edge_count
                if (
                    first_next == second_index
                    or second_next == first_index
                ):
                    continue
                if cls._segments_intersect(
                    polygon[first_index],
                    polygon[first_next],
                    polygon[second_index],
                    polygon[second_next],
                ):
                    return True
        return False

    @classmethod
    def _polygons_intersect(cls, first, second):
        return any(
            cls._segments_intersect(
                first[first_index],
                first[(first_index + 1) % len(first)],
                second[second_index],
                second[(second_index + 1) % len(second)],
            )
            for first_index in range(len(first))
            for second_index in range(len(second))
        )

    @staticmethod
    def _segments_intersect(a, b, c, d):
        def orientation(start, end, point):
            return (
                (end[0] - start[0]) * (point[1] - start[1])
                - (end[1] - start[1]) * (point[0] - start[0])
            )

        return (
            orientation(a, b, c) * orientation(a, b, d) < 0
            and orientation(c, d, a) * orientation(c, d, b) < 0
        )

    @staticmethod
    def _targets(node):
        if isinstance(node, ast.Assign):
            return node.targets
        return [node.target]


if __name__ == '__main__':
    unittest.main()
