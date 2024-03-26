import gym_example
import gymnasium

env = gymnasium.make('PandaEnv-v0')

env.reset()
for _ in range(1000):
    env.render()
    action = [1, 1, 1, 1]
    print("action: ", action)
    env.step(action) # take a random action

env.close()