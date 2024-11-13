from stable_baselines3.common.env_checker import check_env
from icecream import ic

import jumper_env

env = jumper_env.JumperEnv(render=False, show_training=True)
# check_env(env)

episodes = 5

for episode in range(episodes):
    done = False
    obs, _ = env.reset()
    ic(episode)
    while not done:
        action = env.action_space.sample()
        obs, reward, done, truncated, info = env.step(action)
        ic(obs)
