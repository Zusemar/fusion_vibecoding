import ast
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

    @staticmethod
    def _targets(node):
        if isinstance(node, ast.Assign):
            return node.targets
        return [node.target]


if __name__ == '__main__':
    unittest.main()
