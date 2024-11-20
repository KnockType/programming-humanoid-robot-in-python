'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy import matrix, identity, sin, cos

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
                       }
        # joint name: [axis, minrange(rad), maxrange(rad), xlen(mm), ylen(mm), zlen(mm)]
        self.links = {
            'HeadYaw': ['Z', -2.0857, 2.0857, 0, 0, 126.5],
            'HeadPitch': ['Y', -0.6720, 0.5149, 0, 0, 0],
            'LShoulderPitch': ['Y', -2.0857, 2.0857, 0, 98, 100],
            'LShoulderRoll': ['Z', -0.3142, 1.3265, 0, 0, 0],
            'LElbowYaw': ['X', -2.0857, 2.0857, 105, 15, 0],
            'LElbowRoll': ['Z', 1.5446, 0.0349, 0, 0, 0],
            'LWristYaw': ['X', -1.8238, 1.8238, 55.95, 0, 0],
            'LHipYawPitch': ['Z', -1.145303, 0.740810, 0, 50, -85],
            'LHipRoll': ['X', -0.379472, 0.790477, 0, 0, 0],
            'LHipPitch': ['Y', -1.535889, 0.484090, 0, 0, 0],
            'LKneePitch': ['Y', -0.092346, 2.112528, 0, 0, -100],
            'LAnklePitch': ['Y', -1.189516, 0.922747, 0, 0, -102.9],
            'LAnkleRoll': ['X', -0.397880, 0.769001, 0, 0, 0],
            'RHipYawPitch': ['Z', -1.145303, 0.740810, 0, -50, -85],
            'RHipRoll': ['X', -0.790477, 0.379472, 0, 0, 0],
            'RHipPitch': ['Y', -1.535889, 0.484090, 0, 0, 0],
            'RKneePitch': ['Y', -0.103083, 2.120198, 0, 0, -100],
            'RAnklePitch': ['Y', -1.186448, 0.932056, 0, 0, -102.9],
            'RAnkleRoll': ['X', -0.768992, 0.397935, 0, 0, 0],
            'RShoulderPitch': ['Y', -2.0857, 2.0857, 0, -98, 100],
            'RShoulderRoll': ['Z', -0.3142, 1.3265, 0, 0, ],
            'RElbowYaw': ['X', -2.0857, 2.0857, 105, -15, 0],
            'RElbowRoll': ['Z', -1.5446, 0.0349, 0, 0, 0]
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        d = self.links[joint_name]
        if (not d) or joint_angle < d[1] or d[2] < joint_angle:
            return
        s = sin(joint_angle)
        c = cos(joint_angle)
        match d[0]:
            case 'X':
                T = matrix([[1, 0, 0, 0],
                            [0, c, -s, 0],
                            [0, s, c, 0],
                            [0, 0, 0, 1]])
            case 'Y':
                T = matrix([[c, 0, s, 0],
                            [0, 1, 0, 0],
                            [-s, 0, c, 0],
                            [0, 0, 0, 1]])
            case 'Z':
                T = matrix([[c, s, 0, 0],
                            [-s, c, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            case '_':
                return
        T[0, -1] = d[3] / 1000;
        T[1, -1] = d[4] / 1000;
        T[2, -1] = d[5] / 1000;
        return T
        
    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T * Tl
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
