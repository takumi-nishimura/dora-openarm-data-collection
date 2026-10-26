<!-- SPDX-License-Identifier: Apache-2.0 -->
<!-- Added in the takumi-nishimura fork. -->

# Source provenance and fork modifications

This fork starts from [enactic/dora-openarm-data-collection at
e398fa5](https://github.com/enactic/dora-openarm-data-collection/tree/e398fa5656a79c76d12eb123ee3f16f54bff19ac).
The upstream [LICENSE](../LICENSE) is retained unchanged. The additions described
here continue to use Apache-2.0. Identifying this fork as the source of a change
does not identify or transfer copyright ownership.

## Existing material and modifications

Applicable upstream copyright and attribution notices remain in inherited
files. Modified files carry their own change notice:

| File | Fork modification |
| --- | --- |
| `.gitignore` | Exclude generated Python bytecode. |
| `.gitmodules` | Add the official Keyboard submodule. |
| `README.md` | Document Keyboard, model profiles and local verification. |
| `dataflow.yaml` | Document the Keyboard leader selection. |
| `modules/leader-vr.yaml`, `modules/leader-webxr.yaml` | Use the common model profile for IK. |
| `modules/robot-mujoco.yaml` | Share model settings and configure viewer/rendering. |
| `pyproject.toml` | Add the editable Keyboard dependency. |
| `uv.lock` | Resolve that dependency while retaining existing package versions. |

`modules/leader-keyboard.yaml` adapts the upstream leader module structure and
IK connection pattern, including
[`modules/leader-vr.yaml`](https://github.com/enactic/dora-openarm-data-collection/blob/e398fa5656a79c76d12eb123ee3f16f54bff19ac/modules/leader-vr.yaml).
Its inherited Enactic notice is retained, and its adaptation is marked.

`.github/workflows/test.yaml` matches the upstream baseline. The added Keyboard
and model test step was removed; the verification scripts are run locally.

## Additions made in this fork

The following integration and verification scripts were added in this fork
using the existing nodes and package APIs:

- `scripts/model-node`
- `scripts/model_config.py`
- `scripts/inspect-model.py`
- `scripts/test-keyboard-headless.py`
- `scripts/test-model-config.py`

The `Copyright 2026 Enactic, Inc.` line initially placed in these five files
was copied without establishing an ownership basis for these additions.
That unsupported line has been removed. Their Apache-2.0 license notices remain;
no replacement individual, employer or institutional copyright owner is asserted.
This correction does not remove notices from inherited upstream material.

The model profiles and the Keyboard/model verification documentation are also
fork additions. They reference the existing model assets rather than copying
the meshes or node implementations. Their SPDX identifiers state the license
without inventing a copyright owner.

## Generated files and dependencies

`uv.lock` is generated, but its fork change notice must remain in the distributed
file. After an intentional `uv lock` update, restore the following two comments
at the beginning if the tool removed them. Update their description if the
dependency change changes:

```toml
# Modified in the takumi-nishimura fork: resolve the added Keyboard dependency.
# Regenerated lockfile; restore this notice after regeneration (docs/provenance.md).
```

Review the lockfile diff and run `uv lock --check` before committing. Adding
these comments does not change the parsed dependency data. Do not update package
versions just to restore a notice.

The parent and the pinned official Keyboard submodule have no upstream `NOTICE`
file at the versions reviewed here. Submodules and installed dependencies retain
their own LICENSE/NOTICE files; their ownership is not reassigned by this fork.
Any future code copied or adapted from another source needs its provenance and
applicable notices checked individually.

These conventions implement the preservation and modification-notice requirements
in [Apache License 2.0, section 4](https://www.apache.org/licenses/LICENSE-2.0).
