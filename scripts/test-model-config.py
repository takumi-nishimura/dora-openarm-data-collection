#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Added in the takumi-nishimura fork; see docs/provenance.md.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check configuration against existing parsers, geometry, and driver limits."""

import argparse
from importlib.util import find_spec
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from openarm_control import register_common_args
from scipy.spatial.transform import Rotation
import yaml

from model_config import ROOT, ModelProfile, load_profile, node_configuration


class ModelTests(unittest.TestCase):
    def test_repository_relative_xml_uses_same_model(self):
        original = load_profile("pedestal")
        settings = dict(original.settings)
        del settings["asset"]
        settings["xml"] = str(original.xml.relative_to(ROOT))
        local = ModelProfile("local", settings)
        self.assertEqual(local.xml, original.xml)
        self.assertEqual(local.setup().model.nq, original.setup().model.nq)

    def test_profile_path_stays_in_profile_directory(self):
        with self.assertRaises(ValueError):
            load_profile("../pedestal")

    def test_ik_and_simulator_share_model_frames_and_home(self):
        from dora_openarm_mujoco.main import _parse_args

        for name in ("demo", "pedestal"):
            with self.subTest(profile=name):
                profile = load_profile(name)
                ik, _ = node_configuration("dora-openarm-ik", [], profile)
                parser = argparse.ArgumentParser()
                register_common_args(parser)
                ik_args = parser.parse_args(ik[1:])
                simulation, _ = node_configuration("dora-openarm-mujoco", [], profile)
                with patch("sys.argv", simulation):
                    sim_args = _parse_args()
                self.assertEqual(ik_args.xml, sim_args.xml)
                self.assertEqual(ik_args.keyframe, sim_args.keyframe)
                self.assertEqual(ik_args.origin_frame, sim_args.origin_frame)
                self.assertEqual(ik_args.origin_frame_type, sim_args.origin_frame_type)

    def test_keyboard_starts_at_fk_home(self):
        from dora_openarm_keyboard.main import build_parser

        for name in ("demo", "pedestal"):
            profile = load_profile(name)
            command, _ = node_configuration("dora-openarm-keyboard", [], profile)
            args = build_parser().parse_args(command[1:])
            setup = profile.setup()
            for side in ("right", "left"):
                pose = setup.read_ee_pose(side)
                np.testing.assert_allclose(
                    getattr(args, "home_" + side), pose[:3], atol=1e-7
                )
                target = Rotation.from_euler("xyz", args.home_rpy, degrees=True)
                actual = Rotation.from_quat(pose[[4, 5, 6, 3]])
                self.assertLess((target.inv() * actual).magnitude(), 1e-6)

    def test_pedestal_joint_order_roundtrip(self):
        setup = load_profile("pedestal").setup()
        for side in ("right", "left"):
            vector = np.array(
                [
                    0.1,
                    0.05 if side == "right" else -0.05,
                    -0.1,
                    1.3,
                    0.2,
                    -0.15,
                    0.1,
                    -0.3 if side == "right" else 0.3,
                ]
            )
            setup.joint_resolver.set_qpos(setup.data.qpos, vector, side)
            joints, grip = setup.joint_resolver.get_driver(setup.data.qpos, side)
            np.testing.assert_allclose([*joints, grip], vector, atol=1e-9)

    def test_pedestal_arm_limits_match_installed_driver(self):
        spec = find_spec("openarm_driver")
        config_path = (
            Path(next(iter(spec.submodule_search_locations)))
            / "configs"
            / "openarm_pedestal.yaml"
        )
        config = yaml.safe_load(config_path.read_text())
        setup = load_profile("pedestal").setup()
        for side in ("right", "left"):
            limits = [
                setup.model.joint(f"openarm_{side}_joint{i}").range for i in range(1, 8)
            ]
            np.testing.assert_allclose(
                limits, config["joint_limits"][side + "_arm"][:7], atol=1e-9
            )
            self.assertTrue(np.allclose(config["joint_offsets"][side + "_arm"], 0))

    def test_pedestal_mount_geometry(self):
        setup = load_profile("pedestal").setup()
        np.testing.assert_allclose(
            setup.data.site_xpos[setup.origin_id], [0, 0, 0.698], atol=1e-9
        )
        for side, y in (("right", -0.031), ("left", 0.031)):
            body = setup.model.body("openarm_" + side + "_base_link")
            np.testing.assert_allclose(
                setup.data.xpos[body.id], [0, y, 0.698], atol=1e-9
            )
        self.assertEqual(setup.model.nq, 18)
        self.assertNotIn(
            "openarm_lifter_joint",
            [setup.model.joint(i).name for i in range(setup.model.njnt)],
        )

    def test_pedestal_is_default_for_real_pedestal(self):
        with patch.dict("os.environ", {"ROBOT": "pedestal"}, clear=True):
            self.assertEqual(load_profile().name, "pedestal")

    def test_pedestal_control_profile_has_no_camera_rendering(self):
        with patch.dict("os.environ", {}, clear=True):
            _, env = node_configuration(
                "dora-openarm-mujoco", [], load_profile("pedestal")
            )
            self.assertEqual(env["DORA_OPENARM_MUJOCO_RENDER"], "false")
            self.assertEqual(env["DORA_OPENARM_MUJOCO_VIEWER"], "true")

    def test_node_cannot_override_shared_model(self):
        for extra in (
            ["--xml", "other.xml"],
            ["--scene=demo"],
            ["--home-rpy", "0", "0", "0"],
        ):
            with self.assertRaises(ValueError):
                node_configuration("dora-openarm-ik", extra, load_profile("pedestal"))


if __name__ == "__main__":
    unittest.main()
