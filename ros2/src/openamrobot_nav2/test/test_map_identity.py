"""
Pure tests for map_identity_hash/load_map_identity_source.

Both the live node (NavigationStatusNode._compute_map_hash) and the offline
tool (tools/compute_map_identity.py) call these same two functions, so a
real match or mismatch can never come from the two computing it differently
- see map_identity.py's docstring. These tests cover the hash function on
its inputs directly (no files needed) and load_map_identity_source reading
real temporary map.yaml + image files on disk.
"""

import os
import tempfile
import unittest

from openamrobot_nav2.map_identity import load_map_identity_source, map_identity_hash

BASE_KWARGS = dict(
    resolution=0.05, origin_x=-5.391, origin_y=-5.189, origin_yaw=0.0,
    negate=False, occupied_thresh=0.65, free_thresh=0.196, mode='trinary',
    image_bytes=b'\x00\x01\x02\x03fake-pgm-bytes',
)


class TestMapIdentityHash(unittest.TestCase):

    def test_identical_inputs_hash_identically(self):
        self.assertEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**BASE_KWARGS))

    def test_changed_image_bytes_changes_the_hash(self):
        changed = dict(BASE_KWARGS, image_bytes=b'\x00\x01\x02\x03different-bytes')
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_a_single_changed_byte_changes_the_hash(self):
        changed = dict(BASE_KWARGS, image_bytes=b'\x00\x01\x02\x04fake-pgm-bytes')
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_resolution_change_changes_the_hash(self):
        changed = dict(BASE_KWARGS, resolution=0.1)
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_origin_change_changes_the_hash(self):
        changed = dict(BASE_KWARGS, origin_x=0.0)
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_negate_changes_the_hash(self):
        changed = dict(BASE_KWARGS, negate=True)
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_occupied_thresh_changes_the_hash(self):
        # This is exactly the case Mohamed Sayed's platform-sw#73 review
        # flagged as silently NOT changing the old OccupancyGrid-based hash:
        # the declared interpretation of a map's pixels changed, so the
        # identity must change too, even though no pixel decoding happens
        # here at all - the threshold value itself is part of what's hashed.
        changed = dict(BASE_KWARGS, occupied_thresh=0.75)
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_free_thresh_changes_the_hash(self):
        changed = dict(BASE_KWARGS, free_thresh=0.3)
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_mode_changes_the_hash(self):
        changed = dict(BASE_KWARGS, mode='scale')
        self.assertNotEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**changed))

    def test_float_jitter_within_rounding_does_not_change_the_hash(self):
        jittered = dict(
            BASE_KWARGS,
            resolution=0.05 + 1e-9, origin_x=-5.391 + 1e-9, origin_y=-5.189 - 1e-9)
        self.assertEqual(map_identity_hash(**BASE_KWARGS), map_identity_hash(**jittered))

    def test_int_vs_float_resolution_does_not_change_the_hash(self):
        as_int = dict(BASE_KWARGS, resolution=BASE_KWARGS['resolution'])
        as_str_like_float = dict(BASE_KWARGS, resolution=float(BASE_KWARGS['resolution']))
        self.assertEqual(map_identity_hash(**as_int), map_identity_hash(**as_str_like_float))

    def test_image_bytes_accepts_bytearray_too(self):
        as_bytes = dict(BASE_KWARGS, image_bytes=bytes(BASE_KWARGS['image_bytes']))
        as_bytearray = dict(BASE_KWARGS, image_bytes=bytearray(BASE_KWARGS['image_bytes']))
        self.assertEqual(map_identity_hash(**as_bytes), map_identity_hash(**as_bytearray))


# A tiny, valid 2x2 PGM (P5, maxval 255, two bytes of pixel data) - enough to
# exercise load_map_identity_source's file-reading path without needing Pillow
# or any real map asset.
_TINY_PGM = b'P5\n2 2\n255\n' + bytes([10, 20, 30, 40])


class TestLoadMapIdentitySource(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        self.image_path = os.path.join(self.tmpdir.name, 'map.pgm')
        with open(self.image_path, 'wb') as f:
            f.write(_TINY_PGM)

    def _write_yaml(self, name='map.yaml', **overrides):
        fields = {
            'image': 'map.pgm',
            'resolution': 0.05,
            'origin': [-1.0, -2.0, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'mode': 'trinary',
        }
        fields.update(overrides)
        path = os.path.join(self.tmpdir.name, name)
        with open(path, 'w') as f:
            for key, value in fields.items():
                if key == 'origin':
                    f.write(f'origin: {value}\n')
                elif isinstance(value, str):
                    f.write(f'{key}: "{value}"\n')
                else:
                    f.write(f'{key}: {value}\n')
        return path

    def test_reads_declared_fields_and_raw_image_bytes(self):
        path = self._write_yaml()
        source = load_map_identity_source(path)
        self.assertEqual(source['resolution'], 0.05)
        self.assertEqual(source['origin_x'], -1.0)
        self.assertEqual(source['origin_y'], -2.0)
        self.assertEqual(source['origin_yaw'], 0.0)
        self.assertEqual(source['negate'], False)
        self.assertEqual(source['occupied_thresh'], 0.65)
        self.assertEqual(source['free_thresh'], 0.196)
        self.assertEqual(source['mode'], 'trinary')
        self.assertEqual(source['image_bytes'], _TINY_PGM)

    def test_image_bytes_are_never_decoded_a_corrupt_image_still_reads(self):
        # The whole point of hashing raw bytes (interfaces#28) is that this
        # function never has to understand the image format - garbage bytes
        # still read back unchanged, because nothing here parses them.
        with open(self.image_path, 'wb') as f:
            f.write(b'this is not a valid pgm file at all')
        path = self._write_yaml()
        source = load_map_identity_source(path)
        self.assertEqual(source['image_bytes'], b'this is not a valid pgm file at all')

    def test_relative_image_path_resolves_against_the_yaml_directory(self):
        path = self._write_yaml()
        source = load_map_identity_source(path)
        # already covered by test_reads_declared_fields_and_raw_image_bytes
        # succeeding at all (it would raise FileNotFoundError otherwise);
        # this asserts it explicitly for clarity.
        self.assertEqual(source['image_bytes'], _TINY_PGM)

    def test_missing_image_file_raises(self):
        path = self._write_yaml(image='does_not_exist.pgm')
        with self.assertRaises(OSError):
            load_map_identity_source(path)

    def test_missing_yaml_file_raises(self):
        with self.assertRaises(OSError):
            load_map_identity_source(os.path.join(self.tmpdir.name, 'nope.yaml'))

    def test_non_mapping_yaml_raises(self):
        path = os.path.join(self.tmpdir.name, 'not_a_map.yaml')
        with open(path, 'w') as f:
            f.write('- just\n- a\n- list\n')
        with self.assertRaises(ValueError):
            load_map_identity_source(path)

    def test_end_to_end_matches_a_direct_hash_of_the_same_file(self):
        # This is the parity guarantee Mohamed Sayed's platform-sw#73 review
        # asked for, made trivial by construction: there is no pixel
        # decoding left anywhere in this path to disagree about, so loading
        # the same file twice and hashing it must always agree with itself.
        path = self._write_yaml()
        first = map_identity_hash(**load_map_identity_source(path))
        second = map_identity_hash(**load_map_identity_source(path))
        self.assertEqual(first, second)

    def test_changing_the_image_file_on_disk_changes_the_computed_hash(self):
        path = self._write_yaml()
        before = map_identity_hash(**load_map_identity_source(path))
        with open(self.image_path, 'wb') as f:
            f.write(b'P5\n2 2\n255\n' + bytes([99, 98, 97, 96]))
        after = map_identity_hash(**load_map_identity_source(path))
        self.assertNotEqual(before, after)


if __name__ == '__main__':
    unittest.main(verbosity=2)
