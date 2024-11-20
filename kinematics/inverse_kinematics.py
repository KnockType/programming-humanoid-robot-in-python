'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, array, sqrt, matrix, asarray, linalg
from scipy.linalg import pinv
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def from_trans(self, m):
        tx, ty, tz = 0, 0, 0
        if m[0, 0] == 1:
            tx = atan2(m[2, 1], m[1, 1])
        elif m[1, 1] == 1:
            ty = atan2(m[0, 2], m[0, 0])
        elif m[2, 2] == 1:
            tz = atan2(m[1, 0], m[0, 0])
        return asarray([m[-1, 0], m[-1, 1], m[-1, 2], 
                        tx, ty, tz])    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        # jacobian
        joint_angles = { n: self.perception.joint[n] for n in self.chains[effector_name] }
        target = matrix(self.from_trans(transform)).T
        lambda_ = 1
        max_step = 0.1
        for i in range(1000):
            self.forward_kinematics(joint_angles)
            Ts = [self.transforms[n] for n in self.chains[effector_name]]
            Te = matrix([self.from_trans(Ts[-1])]).T
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = matrix([self.from_trans(i) for i in Ts[0:len(self.chains[effector_name])]]).T
            J = target - T
            dT = target - T
            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1 
            d_theta = lambda_ * pinv(J) * e
            for i, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += asarray(d_theta.T)[0][i]
            if linalg.norm(d_theta) < 1e-4:
                break       
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        keys = []
        joints = self.chains[effector_name]
        times = [[0, 5]] * len(joints)
        for n in joints:
            keys.append([
                [self.perception.joint[n], [3, 0, 0]], 
                [self.inverse_kinematics(effector_name, transform)[n], [3, 0, 0]]
                        ])
        self.keyframes = (joints, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
