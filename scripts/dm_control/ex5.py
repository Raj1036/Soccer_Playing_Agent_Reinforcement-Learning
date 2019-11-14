from dm_control import suite
from dm_control import viewer
import numpy as np

# Load one task:
env = suite.load(domain_name="quadruped", task_name="fetch")

# Iterate over a task set:
for domain_name, task_name in suite.BENCHMARKING:
  env = suite.load(domain_name, task_name)

#viewer.launch(env)
# Step through an episode and print out reward, discount and observation.
action_spec = env.action_spec()
time_step = env.reset()
#while not time_step.last():
#viewer.launch(env)
all_actions=[]
while not time_step.last():
  action = np.random.uniform(action_spec.minimum,
                             action_spec.maximum,
                             size=action_spec.shape)
  time_step = env.step(action)
  print(time_step.reward, time_step.discount, time_step.observation)
  all_actions.append(action)

viewer.launch(env,all_actions)
viewer.launch(env)
#viewer.render()
