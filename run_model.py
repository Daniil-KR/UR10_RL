import pprint

from stable_baselines3 import PPO, A2C
from robot_env import RobotEnv
import numpy as np
import time

env = RobotEnv(True)

model_path = "models/1690774904/280000.zip" # PPO
#model_path = "models/1690774931/70000.zip" # A2C

# Загрузка весов модели - PPO или A2C
model = PPO.load(model_path, env, verbose=1)


# Демо
episodes = 10
for ep in range(episodes):
    #env.use_random = False
    #env.target_position = np.array([0.3, 0.3, 0.8])
    obs, _ = env.reset()
    done = False
    while not done:
        #pprint.pprint(obs)
        action, _ = model.predict(obs)
        pprint.pprint(action)
        obs, rewards, done, _, _ = env.step(action)
        time.sleep(0.02)
        if done:
            time.sleep(3)
        print(rewards)


