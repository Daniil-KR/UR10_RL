from stable_baselines3 import PPO, A2C
import os
from robot_env import RobotEnv
import time

# Директории для хранения логов и весов модели
models_dir = f"models/{int(time.time())}/"
logdir = f"logs/{int(time.time())}/"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(logdir):
    os.makedirs(logdir)

# Создание окружения для обучения
env = RobotEnv(False)

env.reset()

# Выбор алгоритма для обучения - PPO или A2C
model = A2C('MlpPolicy', env, verbose=1, tensorboard_log=logdir)

# Процесс обучения
TIMESTEPS = 10000
iters = 0
while True:
    iters += 1
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False, tb_log_name=f"A2C")
    model.save(f"{models_dir}/{TIMESTEPS * iters}")
