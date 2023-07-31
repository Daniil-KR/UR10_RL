import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from math import pi
import pprint


class RobotEnv(gym.Env):
    """Custom Environment that follows gym interface."""
    observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(12,), dtype=np.float32)
    #observation_space = gym.spaces.Dict(dict(
    #    desired_goal=gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype='float32'),
    #    achieved_goal=gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype='float32'),
    #    observation=gym.spaces.Box(-np.inf, np.inf, shape=(12,), dtype='float32'),
    #))
    action_space = gym.spaces.Box(low=-1, high=1, shape=(3,), dtype=float)

    def __init__(self, render):
        super().__init__()

        self.connect(render)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.joint_type = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']

        self.planeId = None
        self.tableId = None
        self.robotId = None
        self.joints = []
        self.links = {}
        self.step_id = None

        self.robotStartPos = [0, 0, 2]
        self.robotStartOrientation = p.getQuaternionFromEuler([0, pi, 0])

        self.initial_joint_values = np.array([0.0, 0.0, 0.0])
        self.joint_values = None
        self.position_bounds = [(-0.5, 0.5), (-0.5, 0.5), (0.8, 1)]
        self.effector_orientation = p.getQuaternionFromEuler([0, 0, pi / 2])
        self.distance_threshold = 0.10

        self.target_x_position = None
        self.target_y_position = None
        self.target_z_position = None
        self.target_position = None
        self.use_random = True

        self.reward_prev = 0

    def step(self, action):
        self.step_id += 1
        self.joint_values += np.array(action) * 0.1
        self.joint_values = np.clip(self.joint_values, -1, 1)

        target_pos = self._delta_to_abs_coordinates(self.joint_values, self.position_bounds)
        self.move_hand(target_pos, self.effector_orientation)

        p.stepSimulation()

        info = self.compute_info(action)

        return self.compute_observation(), self.compute_reward(), self.is_done(), False, info

    def move_hand(self, target_position, effector_orientation):
        joint_poses = p.calculateInverseKinematics(
            self.robotId,
            self.links['robot_wrist_3_joint'],
            target_position,
            effector_orientation,
            maxNumIterations=100,
            residualThreshold=.01
        )

        for joint_id in self.links.values():
            p.setJointMotorControl2(
                self.robotId,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[joint_id-1],
            )

    def is_done(self):
        effector_position, _, _, _, _, _, effector_velocity, effector_angular_velocity = \
            p.getLinkState(self.robotId, linkIndex=self.links['robot_wrist_3_joint'], computeLinkVelocity=True)
        distance = np.linalg.norm(effector_position - self.target_position)
        return self.step_id == 200

    def compute_info(self, last_action):
        effector_position, _, _, _, _, _, effector_velocity, effector_angular_velocity = \
            p.getLinkState(self.robotId, linkIndex=self.links['robot_wrist_3_joint'], computeLinkVelocity=True)
        distance = np.linalg.norm(effector_position - self.target_position)
        return {
            'is_success': distance < self.distance_threshold,
            'last_action': last_action
        }

    def compute_reward(self, achieved_goal=None, desired_goal=None, info=None):
        effector_position, _, _, _, _, _, effector_velocity, effector_angular_velocity = \
            p.getLinkState(self.robotId, linkIndex=self.links['robot_wrist_3_joint'], computeLinkVelocity=True)
        distance = np.linalg.norm(effector_position - self.target_position)
        reward = 1 / distance - self.reward_prev # - np.linalg.norm(effector_velocity)
        self.reward_prev = 1 / distance
        #print(reward)
        return reward

    def reset(self, seed=None, options=None):
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        self.step_id = 0
        self.reward_prev = 0

        # Загрузка моделей робота и стола
        self.planeId = p.loadURDF("plane.urdf")
        self.tableId = p.loadURDF("table/table.urdf")
        robotPath = "robot_ur10.urdf"
        self.robotId = p.loadURDF(robotPath, self.robotStartPos, self.robotStartOrientation)

        # Получение инфо о параметрах робота

        numJoints = p.getNumJoints(self.robotId)
        #print('Robot joints: %d' % numJoints)

        for joint_id in range(numJoints):
            info = p.getJointInfo(self.robotId, joint_id)
            data = {
                'jointID': info[0],
                'jointName': info[1].decode('utf-8'),
                'jointType': self.joint_type[info[2]],
                'jointLowerLimit': info[8],
                'jointUpperLimit': info[9],
                'jointMaxForce': info[10],
                'jointMaxVelocity': info[11]
            }
            #print("JOINT INFO:")
            #pprint.pprint(data)
            self.joints.append(data)
            if data['jointType'] != 'FIXED':
                self.links[data['jointName']] = joint_id

        #pprint.pprint(self.links)

        self.joint_values = self.initial_joint_values

        #self.target_position = np.array([0.5, 0.5, 0.7])

        if self.use_random:
            self.target_x_position = np.random.uniform(self.position_bounds[0][0], self.position_bounds[0][1])
            self.target_y_position = np.random.uniform(self.position_bounds[1][0], self.position_bounds[1][1])
            self.target_z_position = 0.7
            self.target_position = np.array([self.target_x_position, self.target_y_position, self.target_z_position])

        #testObjectId = p.loadURDF("random_urdfs/000/000.urdf", self.target_position[0], self.target_position[1], self.target_position[2])
        testObjectId = p.loadURDF("lego/lego.urdf", self.target_position[0], self.target_position[1],
                                  self.target_position[2])
        p.stepSimulation()

        return self.compute_observation(), self.compute_info("zero_action")

    def render(self):
        pass

    def close(self):
        pass

    def connect(self, render):
        if not render:
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraDistance=2,
                cameraYaw=0,
                cameraPitch=-20,
                cameraTargetPosition=[0, 0, 0.5]
            )

    def compute_observation(self):
        state = np.zeros(3 * 4)
        effector_position, _, _, _, _, _, effector_velocity, effector_angular_velocity = \
            p.getLinkState(self.robotId, linkIndex=self.links['robot_wrist_3_joint'], computeLinkVelocity=True)
        state[:3] = effector_position
        state[3:6] = effector_velocity
        state[6:9] = effector_angular_velocity
        state[9:12] = self.target_position - np.asarray(effector_position)

        #return {'observation': state, 'desired_goal': self.target_position, 'achieved_goal': effector_position}
        return state

    def _delta_to_abs_coordinates(self, values, bounds):
        result = np.zeros_like(values)
        for i, (value, (lower_bound, upper_bound)) in enumerate(zip(values, bounds)):
            result[i] = (value + 1) / 2 * (upper_bound - lower_bound) + lower_bound
        return result


