from dm_control import composer
from dm_control.locomotion.examples import basic_cmu_2019
from dm_control import viewer
import numpy as np
from PIL import Image
import subprocess

#from mujoco_py import load_model_from_path, MjSim, MjViewer
#from absl import app

env = basic_cmu_2019.cmu_humanoid_run_walls()

# Get the `action_spec` describing the control inputs.
action_spec = env.action_spec()

#viewer.launch(env)
# Step through the environment for one episode with random actions

time_step = env.reset()

subprocess.call(['rm','-rf','frames'])
subprocess.call(['mkdir','-p','frames'])
time_step_c=0
while not time_step.last():
  action = np.random.uniform(action_spec.minimum, action_spec.maximum,
                             size=action_spec.shape)
  time_step = env.step(action)
#  print(dir(time_step))
  
  image_data=env.physics.render(height=480,width=480)
  img=Image.fromarray(image_data,'RGB')
  img.save("frames/frame-%.10d.png"%time_step_c)
  time_step_c+=1

  print("reward = {}, discount = {}, observations = {}.".format(
  time_step.reward, time_step.discount, time_step.observation))

viewer.launch(environment_loader=basic_cmu_2019.cmu_humanoid_run_walls)

#subprocess.call(['ffmpeg','-framerate','50','-y','-i','frames/frame-%010d.png','-r','30','-pix_fmt','yuv420p','video_name.mp4'])
