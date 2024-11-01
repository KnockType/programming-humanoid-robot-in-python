'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, wipe_forehead, rightBellyToStand, leftBackToStand, leftBellyToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.init_t = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        if not self.init_t:
            self.init_t = perception.time
        ctime = perception.time - self.init_t
        names, times, keys = keyframes
        for idx, joint in enumerate(names):
            jtimes = times[idx]
            jkeys = keys[idx]
            current = None
            for i in range(len(jtimes) - 1):
                if jtimes[i] <= ctime and ctime < jtimes[i + 1]:
                    current = i
            if current is None:
                continue
            t0 = jtimes[current]
            t1 = jtimes[current + 1]
            h1 = jkeys[current][2]
            h2 = jkeys[current + 1][1]
            P0 = jkeys[current][0]
            P3 = jkeys[current + 1][0]
            P1 = P0 + h1[2]
            P2 = P3 + h2[2]
            i = (ctime - t0) / (t1 - t0)
            B = (1 - i)**3 * P0 + 3 * (1 - i)**2 * i * P1 + 3 * (1 - i) * i**2 * P2 + i**3 * P3
            target_joints[joint] = B
        # Avoid error in think function
        if "LHipYawPitch" not in target_joints:
            target_joints["LHipYawPitch"] = perception.joint["LHipYawPitch"]
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
