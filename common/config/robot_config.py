import json
from pathlib import Path
import numpy as np

_HERE = Path(__file__).resolve().parent
PROJECT_ROOT = _HERE.parents[1]
ROBOT_CONFIG_JSON = (_HERE / 'robot_config.json').resolve()
LEGACY_CONFIG_JSON = (_HERE / 'config.json').resolve()
WAVEGO_DEPLOY_YAML = (PROJECT_ROOT / 'rl' / 'config' / 'wavego_deploy_config.yaml').resolve()
CONFIG_FILE = str(ROBOT_CONFIG_JSON if ROBOT_CONFIG_JSON.exists() else LEGACY_CONFIG_JSON)

class RobotConfig:
    def __init__(self):
        with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
            self.data = json.load(f)
            
    @property
    def L1(self): return self.data['geometry']['l1']
    @property
    def L2(self): return self.data['geometry']['l2']
    @property
    def L3(self): return self.data['geometry']['l3']
    @property
    def LENGTH(self): return self.data['geometry']['length']
    @property
    def WIDTH(self): return self.data['geometry']['width']
    
    @property
    def OFFSET_HIP(self): return np.radians(self.data['offsets']['hip'])
    @property
    def OFFSET_KNEE(self): return np.radians(self.data['offsets']['knee'])
    
    @property
    def LIMITS(self): 
        return {k: tuple(v) for k, v in self.data['limits'].items()}
    
    @property
    def SPEED_LIMIT(self): return self.data['motion']['speed_limit']
    @property
    def ACCELERATION(self): return self.data['motion']['acceleration']
    
    def get_dir(self, servo_id):
        # JSON keys are strings
        return self.data['directions'].get(str(servo_id), 1)

    def get_offset(self, servo_id):
        offsets = self.data.get('servo_offsets', {})
        return int(offsets.get(str(servo_id), 0))

# 单例实例
cfg = RobotConfig()
