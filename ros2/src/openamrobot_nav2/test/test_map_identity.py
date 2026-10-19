"""
Tests for the shared map-identity hash (interfaces#27 / platform-sw#36).

Pure function, no ROS dependency - this is what lets the offline tool
(tools/compute_map_identity.py) and the live node (navigation_status_node's
_map_identity_hash) agree on the same value without either one importing
rclpy or the other.
"""

from openamrobot_nav2.map_identity import map_identity_hash

BASE_KWARGS = dict(
    resolution=0.05, width=4, height=3,
    origin_x=1.0, origin_y=2.0, origin_yaw_z=0.0, origin_yaw_w=1.0,
    cell_data=[0, 0, -1, 100, 0, 0, -1, 100, 0, 0, -1, 100],
)


def test_identical_inputs_hash_identically():
    assert map_identity_hash(**BASE_KWARGS) == map_identity_hash(**BASE_KWARGS)


def test_a_single_changed_cell_changes_the_hash():
    changed = dict(BASE_KWARGS, cell_data=[0, 0, -1, 100, 0, 0, -1, 100, 0, 0, -1, 99])
    assert map_identity_hash(**BASE_KWARGS) != map_identity_hash(**changed)


def test_resolution_change_changes_the_hash():
    changed = dict(BASE_KWARGS, resolution=0.10)
    assert map_identity_hash(**BASE_KWARGS) != map_identity_hash(**changed)


def test_origin_change_changes_the_hash():
    changed = dict(BASE_KWARGS, origin_x=1.5)
    assert map_identity_hash(**BASE_KWARGS) != map_identity_hash(**changed)


def test_size_change_changes_the_hash_even_with_same_cell_count():
    # 4x3 vs 3x4: same number of cells, different shape - must not collide
    reshaped = dict(BASE_KWARGS, width=3, height=4)
    assert map_identity_hash(**BASE_KWARGS) != map_identity_hash(**reshaped)


def test_float_jitter_within_rounding_precision_does_not_change_the_hash():
    # two float representations of "the same" resolution/origin that differ
    # only past the 6-decimal rounding - must hash the same, or a map
    # reloaded from disk with slightly different float formatting would
    # spuriously MISMATCH against its own declared identity
    jittered = dict(
        BASE_KWARGS,
        resolution=0.05 + 1e-9,
        origin_x=1.0 + 1e-9,
    )
    assert map_identity_hash(**BASE_KWARGS) == map_identity_hash(**jittered)


def test_negative_cell_values_do_not_crash_and_are_distinguishable():
    # -1 (unknown) must hash differently from 0 (free) at the same position
    unknown = dict(BASE_KWARGS, cell_data=[-1] * 12)
    free = dict(BASE_KWARGS, cell_data=[0] * 12)
    assert map_identity_hash(**unknown) != map_identity_hash(**free)


def test_accepts_a_plain_list_or_any_int_iterable():
    # the live node's grid.data is an array.array('b', ...); the offline
    # tool's cell_data is a plain Python list - both must work the same way
    as_list = dict(BASE_KWARGS, cell_data=list(BASE_KWARGS['cell_data']))
    as_tuple = dict(BASE_KWARGS, cell_data=tuple(BASE_KWARGS['cell_data']))
    assert map_identity_hash(**as_list) == map_identity_hash(**as_tuple)
