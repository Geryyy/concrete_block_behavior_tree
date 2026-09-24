"""Static checks on the sim DDS profile.

Whether a 25-process launch survives is a live-graph, human-run gate (PRD §14) --
verify.sh never starts enough participants to see it. What is pinnable offline is
the wiring: the env hook points at a file that actually ships, that file raises
the participant-index ceiling, and it does not inherit the machine-local file's
peer pinning.
"""

from pathlib import Path
from xml.etree import ElementTree

PACKAGE = "concrete_block_behavior_tree"
PACKAGE_DIR = Path(__file__).resolve().parents[1]
HOOK = PACKAGE_DIR / "env-hooks" / f"{PACKAGE}.dsv.in"
CONFIG = PACKAGE_DIR / "config" / "cyclonedds_sim.xml"
NS = {"c": "https://cdds.io/config"}


def _hook_uri_entry():
    for line in HOOK.read_text().splitlines():
        fields = line.split(";")
        if len(fields) == 3 and fields[1] == "CYCLONEDDS_URI":
            return fields[0], fields[2]
    raise AssertionError("env hook sets no CYCLONEDDS_URI")


def test_hook_points_at_the_committed_sim_config():
    verb, value = _hook_uri_entry()
    # set, not set-if-unset, would clobber a hardware bench's exported URI.
    assert verb == "set-if-unset"
    # Relative to the install prefix; resolved by ament only if the file exists.
    assert value == f"share/{PACKAGE}/{CONFIG.relative_to(PACKAGE_DIR)}"
    assert CONFIG.is_file()


def test_ceiling_is_raised_and_discovery_left_at_the_default():
    domain = ElementTree.parse(CONFIG).getroot().find("c:Domain", NS)
    ceiling = domain.find("c:Discovery/c:MaxAutoParticipantIndex", NS)
    # ~25 launch processes plus whatever gets attached to the running chain.
    assert int(ceiling.text) >= 30
    assert domain.find("c:General/c:AllowMulticast", NS) is None
    assert domain.find("c:Discovery/c:Peers", NS) is None
