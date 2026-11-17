# Copyright 2026 OpenAMRobot contributors
# SPDX-License-Identifier: MIT
"""Every clock read in openamrobot_sil is listed in config/clock_inventory.yaml with a domain.

The clock contract (docs/architecture/sil-topology.md section 6) assigns each timer a clock domain.
This test keeps the inventory and the source in step: a timer added or removed without an
inventory change fails here. It checks text, not timing behaviour.
"""
import pathlib
import re

import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
INVENTORY = PKG / 'config' / 'clock_inventory.yaml'
SOURCE = PKG / 'openamrobot_sil'
EXCLUDED = ('vendor/can_sil/tests/',)
PATTERNS = {
    'time.monotonic': r'time\.monotonic\(',
    'time.time': r'time\.time\(',
    'time.sleep': r'time\.sleep\(',
    'device_timer': r'(?<!def )device_timer\(',
    'health_timer': r'(?<!def )health_timer\(',
    'create_timer': r'create_timer\(',
    'get_clock': r'get_clock\(\)',
    # reads of an injected clock of the contract (sim_time.py interface)
    'clock.now': r'clock\.now\(\)',
    'clock.snapshot': r'clock\.snapshot\(\)',
    'Stamp': r'(?<!class )\bStamp\(',
    'is_fresh': r'(?<!def )is_fresh\(',
}
DOMAINS = {'gazebo', 'renode', 'ros', 'steady'}
RULES = {'C1', 'C2', 'C3', 'C4', 'C5', 'C6'}


def _inventory():
    return yaml.safe_load(INVENTORY.read_text())


def _source_counts():
    found = {}
    for path in sorted(SOURCE.rglob('*.py')):
        rel = path.relative_to(PKG).as_posix()
        if any(part in rel for part in EXCLUDED):
            continue
        text = path.read_text()
        counts = {k: len(re.findall(v, text)) for k, v in PATTERNS.items()}
        counts = {k: v for k, v in counts.items() if v}
        if counts:
            found[rel] = counts
    return found


def test_every_clock_read_is_in_the_inventory():
    listed = {f: entry['counts'] for f, entry in _inventory()['files'].items()}
    assert _source_counts() == listed


def test_every_site_names_a_domain_rule_and_status():
    for f, entry in _inventory()['files'].items():
        assert entry['sites'], f
        for site in entry['sites']:
            assert site['current'] in DOMAINS, (f, site)
            assert site['target'] in DOMAINS, (f, site)
            assert site['rule'] in RULES, (f, site)
            expected = 'conforms' if site['current'] == site['target'] else 'to_change'
            assert site['status'] == expected, (f, site)
            if site['status'] == 'to_change':
                assert site.get('needs'), (f, site)


def test_contract_section_names_the_inventory():
    doc = PKG.parents[2] / 'docs' / 'architecture' / 'sil-topology.md'
    text = doc.read_text()
    assert 'config/clock_inventory.yaml' in text
    for rule in sorted(RULES):
        assert '**%s**' % rule in text, rule
