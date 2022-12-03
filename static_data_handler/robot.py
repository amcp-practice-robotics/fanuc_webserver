import numpy as np
np.set_printoptions(suppress=True)
from typing import Dict, List, Optional, Callable

class Robot:
    def __init__(self,
                 dh_config: Dict[str, List[float]],
                 angle_transformers: Optional[Dict[str, Callable[[float, List[float]], float]]] = None):
        assert set(dh_config.keys()) == {'alpha', 'theta_0', 'a', 'd'}, 'Wrong config'
        assert len(set(map(len, dh_config.values()))) == 1, "Number" #https://www.researchgate.net/profile/Seungwoo-Noh-2/publication/265394265/figure/tbl1/AS:670496492363811@1536870229905/Forward-kinematics-of-the-system-D-H-parameters.png of values for each param must stay the same"
        # TODO theta params should better be like in https://www.researchgate.net/profile/Seungwoo-Noh-2/publication/265394265/figure/tbl1/AS:670496492363811@1536870229905/Forward-kinematics-of-the-system-D-H-parameters.png
        self.dh_config = dh_config
        self.num_of_joints = len(dh_config['alpha'])

        if angle_transformers is None:
            angle_transformers = {}
        self.angle_transformers = angle_transformers

    def _dh_matrix_from_joint(self, joint_idx: int, joint_angles: List[float]) -> np.ndarray:
        if f'j{joint_idx + 1}' in self.angle_transformers:
            theta = self.angle_transformers[f'j{joint_idx + 1}'](joint_angles)
        else:
            theta = joint_angles[joint_idx]

        theta = np.radians(theta)
        theta += self.dh_config['theta_0'][joint_idx]
        cth = np.cos(theta)
        sth = np.sin(theta)
        d = self.dh_config['d'][joint_idx]
        a = self.dh_config['a'][joint_idx]
        alpha = self.dh_config['alpha'][joint_idx]
        calpha = np.cos(alpha)
        salpha = np.sin(alpha)
        return np.array([
            [cth, -sth * calpha, sth * salpha, a * cth],
            [sth, cth * calpha, -cth * salpha, a * sth],
            [0, salpha, calpha, d],
            [0, 0, 0, 1]
        ])

    def forward(self, joint_angles: List[float]) -> np.ndarray:
        matrices = [self._dh_matrix_from_joint(idx, joint_angles) for idx in range(len(joint_angles))]
        homogenous = np.linalg.multi_dot(matrices)
        R = homogenous[:3, :3]
        p = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
        cbeta = np.cos(p)
        r = np.arctan2(R[1, 0] / cbeta, R[0, 0] / cbeta)
        w = np.arctan2(R[2, 1] / cbeta, R[2, 2] / cbeta)
        wpr = np.rad2deg(np.array([w, p, r]))
        xyz = homogenous[:3, 3]
        return np.concatenate([xyz, wpr])

    def backward(self, coordinates: List[float]):
        pass


def init_func():
    dh_conf = {
        'theta_0':[0, np.pi/2, 0, 0, 0, 0],
        'alpha': [np.pi/2, 0, -np.pi/2, np.pi/2, -np.pi/2, np.pi],
        'a': [150., 790., 250., 0, 0, 0],
        'd': [0, 0, 0, -835.0, 0, -100],
    }
    robot = Robot(dh_conf, {'j2': (lambda angles: -angles[1]), 
                            'j3': (lambda angles: angles[1] + angles[2])})
    return robot
