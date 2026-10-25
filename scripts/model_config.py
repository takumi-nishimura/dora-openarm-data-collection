# Copyright 2026 Enactic, Inc.
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

"""Resolve the shared model and derive input homes using existing FK."""

from dataclasses import dataclass
import hashlib
from importlib.metadata import version
import os
from pathlib import Path
import re
import warnings

from openarm_control import ArmSetup
from openarm_control.geometry.poses import read_ee_pose
from openarm_mujoco.v2 import asset_path
from scipy.spatial.transform import Rotation
import yaml


ROOT = Path(__file__).resolve().parents[1]
FRAME_OPTIONS = (
    "origin_frame",
    "origin_frame_type",
    "frame_right",
    "frame_left",
    "frame_type_right",
    "frame_type_left",
)


@dataclass
class ModelProfile:
    name: str
    settings: dict

    @property
    def xml(self):
        if "xml" in self.settings:
            return (ROOT / self.settings["xml"]).resolve()
        return Path(asset_path(self.settings["asset"]))

    def setup(self):
        return ArmSetup.from_args(
            xml=str(self.xml),
            mode="bimanual",
            keyframe=self.settings["keyframe"],
            **{key: self.settings[key] for key in FRAME_OPTIONS},
        )

    def common_arguments(self):
        args = ["--xml", str(self.xml), "--keyframe", self.settings["keyframe"]]
        for key in FRAME_OPTIONS:
            args += ["--" + key.replace("_", "-"), self.settings[key]]
        return args

    def keyboard_home(self):
        setup = self.setup()
        poses = {side: setup.read_ee_pose(side) for side in ("right", "left")}
        rotations = {
            side: Rotation.from_quat(pose[[4, 5, 6, 3]]) for side, pose in poses.items()
        }
        if (rotations["right"].inv() * rotations["left"]).magnitude() > 1e-5:
            raise ValueError(
                "The official Keyboard node requires a shared home orientation"
            )
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Gimbal lock detected")
            rpy = rotations["right"].as_euler("xyz", degrees=True)
        return poses, rpy

    def report(self):
        setup = self.setup()
        origin = (
            read_ee_pose(setup.data, setup.origin_id, setup.origin_type)
            if setup.origin_id is not None
            else [0, 0, 0, 1, 0, 0, 0]
        )
        origin_rotation = Rotation.from_quat(
            [origin[4], origin[5], origin[6], origin[3]]
        )
        arms = {}
        for side in ("right", "left"):
            joints, gripper = setup.joint_resolver.get_driver(setup.data.qpos, side)
            body = setup.model.body("openarm_" + side + "_base_link")
            joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
            arms[side] = {
                "base_world_position_m": setup.data.xpos[body.id].tolist(),
                "base_world_quaternion_wxyz": setup.data.xquat[body.id].tolist(),
                "home_pose": setup.read_ee_pose(side).tolist(),
                "home_driver_qpos": [*joints.tolist(), float(gripper)],
                "joint_names": joint_names,
                "joint_axes_local": [
                    setup.model.joint(name).axis.tolist() for name in joint_names
                ],
                "joint_limits_rad": [
                    setup.model.joint(name).range.tolist() for name in joint_names
                ],
            }
        return {
            "profile": self.name,
            "xml": str(self.xml),
            "xml_sha256": hashlib.sha256(self.xml.read_bytes()).hexdigest(),
            "settings": self.settings,
            "versions": {
                name: version(name)
                for name in (
                    "openarm-mujoco",
                    "openarm-control",
                    "openarm-driver",
                    "mujoco",
                )
            },
            "origin_world_position_m": list(map(float, origin[:3])),
            "origin_world_rotation_matrix": origin_rotation.as_matrix().tolist(),
            "nq": setup.model.nq,
            "nv": setup.model.nv,
            "cameras": [setup.model.camera(i).name for i in range(setup.model.ncam)],
            "arms": arms,
        }


def load_profile(name=None):
    name = (
        name
        or os.environ.get("OPENARM_MODEL")
        or ("pedestal" if os.environ.get("ROBOT") == "pedestal" else "demo")
    )
    if not re.fullmatch(r"[a-z][a-z0-9_-]*", name):
        raise ValueError(f"Invalid model profile: {name!r}")
    path = ROOT / "profiles" / (name + ".yaml")
    settings = yaml.safe_load(path.read_text())
    if (
        not isinstance(settings, dict)
        or sum(key in settings for key in ("asset", "xml")) != 1
    ):
        raise ValueError(f"{path}: specify exactly one of asset or xml")
    profile = ModelProfile(name, settings)
    if not profile.xml.is_file():
        raise FileNotFoundError(profile.xml)
    return profile


def node_configuration(command, extra, profile):
    """Build arguments for the official entrypoint, without implementing a node."""
    protected = {
        "--xml",
        "--scene",
        "--keyframe",
        "-k",
        "--origin-frame",
        "--origin-frame-type",
        "--frame-right",
        "--frame-left",
        "--frame-type-right",
        "--frame-type-left",
        "--home-right",
        "--home-left",
        "--home-rpy",
    }
    if any(arg.split("=", 1)[0] in protected for arg in extra):
        raise ValueError("Set model, frames, and home in profiles/, not node arguments")
    env = os.environ.copy()
    if command in ("dora-openarm-ik", "dora-openarm-fk"):
        args = profile.common_arguments()
    elif command == "dora-openarm-mujoco":
        args = [
            "--xml",
            str(profile.xml),
            "--keyframe",
            profile.settings["keyframe"],
            "--origin-frame",
            profile.settings["origin_frame"],
            "--origin-frame-type",
            profile.settings["origin_frame_type"],
        ]
        env["DORA_OPENARM_MUJOCO_VIEWER"] = env.get("MUJOCO_VIEWER") or "true"
        env["DORA_OPENARM_MUJOCO_RENDER"] = (
            env.get("MUJOCO_RENDER") or str(profile.settings["render"]).lower()
        )
    elif command == "dora-openarm-keyboard":
        poses, rpy = profile.keyboard_home()
        args = [
            "--home-right",
            *map(str, poses["right"][:3]),
            "--home-left",
            *map(str, poses["left"][:3]),
            "--home-rpy",
            *map(str, rpy),
        ]
    else:
        raise ValueError(f"Unsupported model-aware entrypoint: {command}")
    return [command, *args, *extra], env
