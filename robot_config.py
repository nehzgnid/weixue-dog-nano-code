import json
import os
import numpy as np

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'config.json')

class RobotConfig:
    def __init__(self):
        with open(CONFIG_FILE, 'r') as f:
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

# 单例实例
cfg = RobotConfig()
