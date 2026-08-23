from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LearnCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = json.loads((ROOT / "curriculum/modules.json").read_text(encoding="utf-8"))

    def create_fixture(self, fixture: Path) -> None:
        shutil.copytree(ROOT / "bin", fixture / "bin")
        shutil.copytree(ROOT / "curriculum", fixture / "curriculum")
        manifest = json.loads((fixture / "curriculum/modules.json").read_text(encoding="utf-8"))
        for module in manifest["modules"]:
            source = ROOT / module["folder"]
            target = fixture / module["folder"]
            target.mkdir(parents=True, exist_ok=True)
            for name in (
                "README.md",
                "lesson.md",
                "walkthrough.md",
                "checks.md",
                "run_checks.m",
            ):
                if (source / name).exists():
                    shutil.copy2(source / name, target / name)

    def run_cli_in_fixture(
        self, fixture: Path, *args: str
    ) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        return subprocess.run(
            [str(fixture / "bin/learn"), *args],
            cwd=fixture,
            text=True,
            capture_output=True,
            env=environment,
            timeout=10,
        )

    def run_cli(self, *args: str) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Path(temporary) / "repo"
            self.create_fixture(fixture)
            return self.run_cli_in_fixture(fixture, *args)

    def test_status_and_list(self):
        status = self.run_cli("status")
        self.assertEqual(status.returncode, 0, status.stderr)
        implemented = sum(
            module["status"] == "implemented" for module in self.manifest["modules"]
        )
        self.assertIn(
            f"{self.manifest['module_count']} total, {implemented} implemented",
            status.stdout,
        )
        listing = self.run_cli("list")
        self.assertEqual(listing.returncode, 0, listing.stderr)
        self.assertEqual(
            len([line for line in listing.stdout.splitlines() if line.strip()]),
            self.manifest["module_count"],
        )

    def test_all_manifest_implemented_modules_start_and_check(self):
        implemented_ids = [
            module["id"]
            for module in self.manifest["modules"]
            if module["status"] == "implemented"
        ]
        for module_id in implemented_ids:
            with self.subTest(module=module_id):
                result = self.run_cli("start", module_id)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f"{module_id} —", result.stdout)
                self.assertIn("Status: implemented", result.stdout)
                self.assertIn("Guiding question:", result.stdout)

                check = self.run_cli("check", module_id)
                self.assertEqual(check.returncode, 0, check.stderr)
                self.assertIn(f"run_module_checks('{module_id}')", check.stdout)

    def test_current_frontier_scaffold_refuses(self):
        scaffolded = next(
            (module for module in self.manifest["modules"] if module["status"] == "scaffolded"),
            None,
        )
        if scaffolded is None:
            self.skipTest("the canonical manifest has no remaining scaffolded module")
        scaffold = self.run_cli("start", scaffolded["id"])
        self.assertEqual(scaffold.returncode, 2)
        self.assertIn("Activate its governed implementation batch", scaffold.stdout)

    def test_rejected_scaffold_does_not_replace_resumable_selection(self):
        scaffolded = next(
            (module for module in self.manifest["modules"] if module["status"] == "scaffolded"),
            None,
        )
        if scaffolded is None:
            self.skipTest("the canonical manifest has no remaining scaffolded module")

        with tempfile.TemporaryDirectory() as temporary:
            fixture = Path(temporary) / "repo"
            self.create_fixture(fixture)

            selected = self.run_cli_in_fixture(fixture, "start", "P02")
            self.assertEqual(selected.returncode, 0, selected.stderr)

            rejected = self.run_cli_in_fixture(fixture, "start", scaffolded["id"])
            self.assertEqual(rejected.returncode, 2, rejected.stderr)

            resumed = self.run_cli_in_fixture(fixture, "continue")
            self.assertEqual(resumed.returncode, 0, resumed.stderr)
            self.assertIn("P02 —", resumed.stdout)

            state = json.loads(
                (fixture / ".learning/progress.json").read_text(encoding="utf-8")
            )
            self.assertEqual(state["current"], "P02")


if __name__ == "__main__":
    unittest.main()
