import numpy as np
np.set_printoptions(suppress=True)
from typing import Dict, List, Optional, Callable, Sequence

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

    def _get_transform_matrix(self, joint_angles: List[float]) -> np.ndarray:
        """
        Calculate 4x4 transformation matrix for n-th joint, n - length of joint_angles
        :param joint_angles: angles (in radians!)
        :return: 4x4 matrix
        """
        matrices = [self._dh_matrix_from_joint(idx, joint_angles) for idx in range(len(joint_angles))]
        return np.linalg.multi_dot(matrices)
    
    def _transform_from_coords(self, xyzwpr: np.ndarray) -> np.ndarray:
        """
        Calculate 4x4 transformation matrix from frame position and orientation
        :param xyzwpr: array of len 6 (wpr are  in radians)
        :return: 4x4 matrix
        """
        x, y, z, *wpr = xyzwpr
        sa, sb, sg = map(np.sin, wpr[::-1])
        ca, cb, cg = map(np.cos, wpr[::-1])
        return np.array([
            [ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg, x],
            [sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg, y],
            [-sb, cb*sg, cb*cg, z],
            [0,0,0,1]
        ])
        
    def _geometric_thetas_23(self, x, y) -> np.ndarray:
        """
        Calculates angles to horizontal
        :return: 2x2 array, each row is: theta_2, theta3
        """
        radius_vector_angle = np.arctan2(y, x)
        a = self.dh_config['a'][1]
        b = np.hypot(self.dh_config['a'][2], -self.dh_config['d'][3])
        rho = np.hypot(x, y)
        triangle_angle = np.arccos((a**2 + rho**2 - b**2)/(2*a*rho))
        thetas_2 = np.array([triangle_angle, -triangle_angle]) + radius_vector_angle
        offset_th_3 = np.arctan(self.dh_config['a'][2]/self.dh_config['d'][3])
        thetas_3 = np.arctan2(y - a*np.sin(thetas_2), x - a*np.cos(thetas_2)) + offset_th_3
        return np.vstack([thetas_2, thetas_3]).T
    
    def _geometric_thetas_123(self, x, y, z) -> np.ndarray:
        """
        :return: array of shape 4x3, each row - theta1, theta2, theta3
        """
        thetas_1 = np.array([np.arctan2(y, x), np.arctan2(-y, -x)])
        rho_plane = np.hypot(x, y)
        thetas_23 = np.vstack([
            self._geometric_thetas_23(rho_plane - self.dh_config['a'][0], z),
            self._geometric_thetas_23(-rho_plane - self.dh_config['a'][0], z),
        ])
        return np.hstack([np.repeat(thetas_1, 2).reshape(-1,1), thetas_23])
    
    def _zyz_angles_from_R(self, R):
        sin_b = np.hypot(R[0, 2], R[1, 2])*np.array([1, -1])
        alpha = np.arctan2(R[1, 2]/sin_b, R[0, 2]/sin_b)
        beta = np.arctan2(sin_b, R[2, 2])
        gamma = np.arctan2(R[2, 1]/sin_b, -R[2, 0]/sin_b)
        return np.vstack([alpha, -beta, gamma]).T
   

    def forward(self, joint_angles: Sequence[float]) -> np.ndarray:
        """
        :param joint_angles: angles array of len 6 (in degrees)
        :return: xyzwpr (wpr in degrees)
        """
        joint_angles = np.array(joint_angles)
        joint_angles = np.deg2rad(joint_angles)
        homogenous = self._get_transform_matrix(joint_angles)
        R = homogenous[:3, :3]
        p = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
        cbeta = np.cos(p)
        r = np.arctan2(R[1, 0] / cbeta, R[0, 0] / cbeta)
        w = np.arctan2(R[2, 1] / cbeta, R[2, 2] / cbeta)
        wpr = np.rad2deg(np.array([w, p, r]))
        xyz = homogenous[:3, 3]
        return np.concatenate([xyz, wpr])

    def inverse(self, xyzwpr: Sequence[float], theta_2_function=(lambda t: -t + np.pi/2)) -> np.ndarray:
        """
        :param xyzwpr: array of len 6 (wpr are  in degrees)
        :return: angles array of len 6 (in degrees)
        """
        xyzwpr = np.array(xyzwpr)
        xyzwpr[3:] = np.deg2rad(xyzwpr[3:])
        transform_matrix = self._transform_from_coords(xyzwpr)
        x, y, z, *_ = transform_matrix@np.array([0, 0, self.dh_config['d'][5], 1])
        thetas_123 = self._geometric_thetas_123(x, y, z)
        thetas_123[:, 1] = theta_2_function(thetas_123[:, 1])
        
        thetas_456 = []
        R_final = transform_matrix[:3, :3]
        # print(f'needed {x=}, {y=}, {z=}')
        for thetas_123_variation in thetas_123:
            # T = self._get_transform_matrix(thetas_123_variation)
            # print(f"intersection: {T@np.array([0, 0, self.dh_config['d'][3], 1])}")
            R_3 = self._get_transform_matrix(thetas_123_variation)[:3, :3]
            R = R_3.T @ R_final @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            thetas_456.append(self._zyz_angles_from_R(R))
        thetas_456 = np.vstack(thetas_456)
        
        return np.rad2deg(np.hstack([np.repeat(thetas_123, 2, axis=0), thetas_456]))


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