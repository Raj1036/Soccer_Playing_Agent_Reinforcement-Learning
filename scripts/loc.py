
from dm_control import composer
from dm_control.locomotion.examples import basic_cmu_2019
from dm_control import viewer
import numpy as np

# Build an example environment.
env = basic_cmu_2019.cmu_humanoid_run_walls()
viewer.launch(environment_loader=basic_cmu_2019.cmu_humanoid_run_walls)

action_spec = env.action_spec()

# Step through the environment for one episode with random actions.
time_step = env.reset()
while not time_step.last():
  action = np.random.uniform(action_spec.minimum, action_spec.maximum,
                             size=action_spec.shape)
  time_step = env.step(action)
  print("reward = {}, discount = {}, observations = {}.".format(
      time_step.reward, time_step.discount, time_step.observation))

#viewer.launch(environment_loader=basic_cmu_2019.cmu_humanoid_run_walls)
