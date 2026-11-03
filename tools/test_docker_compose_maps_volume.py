"""Verify the openamr service mounts a persistent maps volume at /data/maps."""
from pathlib import Path

import yaml

COMPOSE_FILE = Path(__file__).resolve().parent.parent / "docker-compose.yml"
VOLUME_NAME = "openamrobot_maps"
MOUNT_TARGET = "/data/maps"


def _load_compose():
    with COMPOSE_FILE.open() as f:
        return yaml.safe_load(f)


def test_openamr_service_mounts_maps_volume():
    compose = _load_compose()
    service_volumes = compose["services"]["openamr"]["volumes"]
    assert any(
        v == f"{VOLUME_NAME}:{MOUNT_TARGET}:rw" or v.startswith(f"{VOLUME_NAME}:{MOUNT_TARGET}:")
        for v in service_volumes
    ), f"expected a {VOLUME_NAME} mount at {MOUNT_TARGET} in services.openamr.volumes, got {service_volumes}"


def test_maps_volume_is_declared_as_named_volume():
    compose = _load_compose()
    assert VOLUME_NAME in compose["volumes"], (
        f"expected top-level volumes to declare {VOLUME_NAME}, got {list(compose['volumes'])}"
    )
