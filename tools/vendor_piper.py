"""Import exact official model bytes from a pinned local Git checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

COMMIT = "017ffefa64511bc6325bd77ddc4e16065c152051"
REPOSITORY = "https://github.com/agilexrobotics/piper_ros"
DESTINATION = Path(__file__).resolve().parents[1] / "src/robot_pick_place_agent/assets/piper"


def vendor(checkout):
    files = {
        "LICENSE": "upstream/LICENSE",
        "README.MD": "upstream/README.MD",
        "src/piper_description/package.xml": "upstream/package.xml",
        "src/piper_description/mujoco_model/piper_description.xml": "upstream/piper_description.xml",
        "src/piper_description/urdf/piper_description.urdf": "upstream/piper_description.urdf",
        "src/piper_description/urdf/piper_description_old.urdf": "upstream/piper_description_old.urdf",
    }
    for name in ["base_link", "gripper_base", *[f"link{i}" for i in range(1, 9)]]:
        files[f"src/piper_description/meshes/{name}.STL"] = f"meshes/{name}.STL"
    manifest = {"repository": REPOSITORY, "branch": "humble", "commit": COMMIT,
                "license": "Root MIT; description/package.xml still contains TODO license declaration",
                "files": []}
    for source, local in files.items():
        data = subprocess.check_output(["git", "-C", str(checkout), "show", f"{COMMIT}:{source}"])
        target = DESTINATION / local
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        manifest["files"].append({"source": source, "local": local, "sha256": hashlib.sha256(data).hexdigest()})
    (DESTINATION / "source.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Imported {len(files)} unchanged files from {COMMIT}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkout", type=Path)
    vendor(parser.parse_args().checkout)
