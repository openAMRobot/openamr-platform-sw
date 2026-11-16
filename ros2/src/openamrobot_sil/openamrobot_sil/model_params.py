"""Parameters with provenance for the battery and power model (config/power_model.yaml).

Each parameter is a mapping with the fields in FIELDS. ``value`` is the robot value or null;
a null value runs on ``sil_default``, which is a SIL run value and never a robot property.
``Params.defaults_in_use()`` names every parameter that runs on its sil_default, so a state
output can show it. ``validate()`` returns the schema problems of a loaded document.
"""
import yaml

CONFIDENCE = ('datasheet', 'supplier_confirmation', 'design_decision', 'cad', 'estimate',
              'unknown')
FIELDS = ('value', 'unit', 'source', 'confidence', 'uncertainty', 'hardware_revision', 'owner',
          'test', 'last_verified', 'measurement_item')


def _is_param(node):
    return isinstance(node, dict) and 'confidence' in node


def _flatten(node, prefix=''):
    out = {}
    for key, sub in node.items():
        name = prefix + str(key)
        if _is_param(sub):
            out[name] = sub
        elif isinstance(sub, dict):
            out.update(_flatten(sub, name + '.'))
    return out


def validate(doc):
    """Schema problems of a parameter document, as a list of strings (empty = valid)."""
    problems = []
    params = _flatten({k: v for k, v in doc.items() if k != 'version'})
    if not params:
        problems.append('no parameters')
    for name, p in params.items():
        for f in FIELDS:
            if f not in p:
                problems.append('%s: missing %s' % (name, f))
        if p.get('confidence') not in CONFIDENCE:
            problems.append('%s: confidence %r not in %s' % (
                name, p.get('confidence'), CONFIDENCE))
        if not p.get('source'):
            problems.append('%s: empty source' % name)
        if not p.get('owner'):
            problems.append('%s: empty owner' % name)
        if p.get('confidence') == 'unknown' and p.get('value') is not None:
            problems.append('%s: confidence unknown but a value is given' % name)
        if p.get('value') is None:
            if 'sil_default' not in p:
                problems.append('%s: null value without sil_default' % name)
            if not p.get('measurement_item'):
                problems.append('%s: null value without a measurement item' % name)
        elif 'sil_default' in p:
            problems.append('%s: sil_default next to a value' % name)
    return problems


class Params:
    """Resolved parameter values. ``get('pack.capacity_ah')``."""

    def __init__(self, doc):
        problems = validate(doc)
        if problems:
            raise ValueError('power model parameters: ' + '; '.join(problems))
        self.meta = _flatten({k: v for k, v in doc.items() if k != 'version'})

    def get(self, name):
        p = self.meta[name]
        return p['value'] if p['value'] is not None else p['sil_default']

    def section(self, prefix):
        """{short name: resolved value} of every parameter directly under prefix."""
        pre = prefix + '.'
        return {k[len(pre):]: self.get(k) for k in self.meta
                if k.startswith(pre) and '.' not in k[len(pre):]}

    def names(self, prefix):
        """Direct child group names under prefix (for converters and branches)."""
        pre = prefix + '.'
        return sorted({k[len(pre):].split('.')[0] for k in self.meta if k.startswith(pre)})

    def defaults_in_use(self):
        return sorted(k for k, p in self.meta.items() if p['value'] is None)

    def by_confidence(self):
        counts = {c: 0 for c in CONFIDENCE}
        for p in self.meta.values():
            counts[p['confidence']] += 1
        return counts


def load(path):
    with open(path, 'r', encoding='utf-8') as f:
        return Params(yaml.safe_load(f))
