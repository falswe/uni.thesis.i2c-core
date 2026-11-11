import itertools
import os
from pathlib import Path

import pytest
from i2c_test_host.peers import load_peers, load_tester

DEVICES_DIR = Path(__file__).resolve().parent.parent / "devices"

# Arbitration tests run against both manifests. All other hardware tests use
# the first manifest without adding a redundant manifest ID to their node IDs.
if manifest_override := os.environ.get("I2C_TEST_MANIFEST"):
    MANIFESTS = [Path(manifest_override)]
    _MANIFEST_IDS = [Path(manifest_override).stem]
else:
    MANIFESTS = [DEVICES_DIR / "peers_2_stms.yaml"]
    _MANIFEST_IDS = ["2_stms"]

# Ordered (controller, target) pairs are enumerated here from the manifest's
# participant set; the manifest only declares who exists and how to reach them.
# A 2-peer manifest yields 2 ordered pairs, a 3-peer manifest yields 6.
_PEER_NAMES = [peer.name for peer in load_peers(MANIFESTS[0])]
_ORDERED_PAIRS = list(itertools.permutations(range(len(_PEER_NAMES)), 2))
_ORDERED_PAIR_IDS = [f"{_PEER_NAMES[i]}->{_PEER_NAMES[j]}" for i, j in _ORDERED_PAIRS]


def _bringup_enabled() -> bool:
    # Set I2C_TEST_BRINGUP=0 to skip bring-up when hardware is already
    # flashed and running (e.g. attached to a live debug session).
    return os.environ.get("I2C_TEST_BRINGUP", "1").strip().lower() not in ("0", "false", "no")


def pytest_generate_tests(metafunc):
    if (
        len(MANIFESTS) > 1
        and "manifest" in metafunc.fixturenames
        and metafunc.definition.get_closest_marker("both_manifests") is not None
    ):
        metafunc.parametrize(
            "manifest",
            MANIFESTS,
            ids=_MANIFEST_IDS,
            indirect=True,
            scope="session",
        )


@pytest.fixture(scope="session")
def manifest(request):
    manifest_path = getattr(request, "param", MANIFESTS[0])
    if not _bringup_enabled():
        yield manifest_path
        return

    from i2c_test_host.bringup import BringupStack, load_bringup_config

    with BringupStack(load_bringup_config(manifest_path)):
        yield manifest_path


@pytest.fixture
def peers(manifest):
    devs = load_peers(manifest)
    for dev in devs:
        dev.open()
    try:
        yield devs
    finally:
        for dev in reversed(devs):
            dev.close()


@pytest.fixture
def dut_pair(peers):
    if len(peers) != 2:
        raise RuntimeError(f"expected exactly 2 DUT peers, got {len(peers)}")
    return tuple(peers)


@pytest.fixture(params=_ORDERED_PAIRS, ids=_ORDERED_PAIR_IDS)
def ordered_pair(peers, request):
    i, j = request.param
    return peers[i], peers[j]


@pytest.fixture
def tester(manifest):
    device = load_tester(manifest)
    if device is not None:
        device.open()
        try:
            yield device
        finally:
            device.close()
    else:
        pytest.skip("no tester node defined in manifest")
