import argparse
import pprint

from stable_baselines3 import PPO, A2C
from typing import List

from robot_env import RobotEnv
import numpy as np
import time


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--coordinates', help='comma delimited list of coordinates input', type=str)
    args = parser.parse_args()
    target_coordinates = [float(item) for item in args.coordinates.split(', ')]

    if len(target_coordinates) > 3:
        assert False, f'Wrong number of coordinates: {len(target_coordinates)}'

    env = RobotEnv(True)

    model_path = "models/1690774904/280000.zip"
    #model_path = "models/1690774904/180000.zip"

    # Загрузка весов модели
    model = PPO.load(model_path, env, verbose=1)
    #model = A2C.load(model_path, env, verbose=1)

    # Демо
    episodes = 1
    for ep in range(episodes):
        env.use_random = False
        env.target_position = np.array(target_coordinates)
        obs, _ = env.reset()
        done = False
        while not done:
            # pprint.pprint(obs)
            action, _ = model.predict(obs)
            pprint.pprint(action)
            obs, rewards, done, _, _ = env.step(action)
            time.sleep(0.05)
            if done:
                time.sleep(5)
            print(rewards)
