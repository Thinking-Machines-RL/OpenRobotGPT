from gymnasium.envs.registration import register

register(
     id="PandaEnv-v0",
     entry_point="gym_prova.envs:PandaEnv",
     max_episode_steps=2000,
)