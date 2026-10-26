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

"""Write a model/driver comparison, optionally with a headless rendered preview."""

import argparse
from importlib.util import find_spec
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=("demo", "pedestal"), default="pedestal")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    if args.preview and not os.environ.get("DISPLAY"):
        os.environ.setdefault("MUJOCO_GL", "egl")

    import numpy as np
    import yaml
    from model_config import ROOT, load_profile

    profile = load_profile(args.model)
    output = args.output or ROOT / "out" / "model-alignment" / profile.name
    output.mkdir(parents=True, exist_ok=True)
    report = profile.report()
    if profile.settings.get("driver_config"):
        spec = find_spec("openarm_driver")
        path = (
            Path(next(iter(spec.submodule_search_locations)))
            / "configs"
            / (profile.settings["driver_config"] + ".yaml")
        )
        driver = yaml.safe_load(path.read_text())
        report["driver_config"] = str(path)
        for side, arm in report["arms"].items():
            limits = driver["joint_limits"][side + "_arm"]
            start = driver["start"]["moves"][0]["position"][side + "_arm"]
            arm.update(
                driver_joint_limits_rad=limits,
                arm_limits_match=bool(
                    np.allclose(arm["joint_limits_rad"], limits[:7], atol=1e-9)
                ),
                driver_offsets_rad=driver["joint_offsets"][side + "_arm"],
                driver_start_qpos=start,
                arm_home_matches_driver_start=bool(
                    np.allclose(arm["home_driver_qpos"][:7], start[:7], atol=1e-6)
                ),
                gripper_model_limits_rad=profile.setup()
                .model.joint(f"openarm_{side}_finger_joint1")
                .range.tolist(),
            )
        report["physical_validation"] = (
            "Not measured by this command; verify physical correspondence, offsets and motion directions in issue #4."
        )
    target = output / "report.json"
    target.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Model report: {target}")

    if args.preview:
        import cv2
        import mujoco

        setup = profile.setup()
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [0.1, 0.0, 0.45]
        camera.distance = 1.7
        camera.azimuth = 135
        camera.elevation = -20
        with mujoco.Renderer(setup.model, height=480, width=640) as renderer:
            renderer.update_scene(setup.data, camera=camera)
            frame = renderer.render()
            image_path = output / "preview.png"
            if not cv2.imwrite(str(image_path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)):
                raise OSError(f"Could not write {image_path}")
            print(f"Preview: {image_path}")


if __name__ == "__main__":
    main()
