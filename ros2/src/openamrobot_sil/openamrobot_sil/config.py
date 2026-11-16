"""Load config/sil.yaml."""
import yaml

from openamrobot_sil.ctl017_gazebo import PermitConfig


def load(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def permit_config(cfg):
    """PermitConfig of the lift section, or None when the gate is disabled."""
    p = cfg['lift'].get('permit') or {}
    if not p.get('enabled', False):
        return None
    return PermitConfig.from_dict(p)
