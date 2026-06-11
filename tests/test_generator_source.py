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

    @staticmethod
    def _targets(node):
        if isinstance(node, ast.Assign):
            return node.targets
        return [node.target]


if __name__ == '__main__':
    unittest.main()
