"""
Shows how to toss a capsule to a container.

Stolen from https://raw.githubusercontent.com/openai/mujoco-py/master/examples/tosser.py
"""
from mujoco_py import load_model_from_path, MjSim, MjViewer
import os

model = load_model_from_path("xmls/new_humanoid.xml")
sim = MjSim(model)

viewer = MjViewer(sim)


sim_state = sim.get_state()

while True:
    sim.set_state(sim_state)

    for i in range(1000):
        if i < 150:
            sim.data.ctrl[:] = 0.0
        else:
            sim.data.ctrl[:] = -1.0
        sim.step()
        viewer.render()

    if os.getenv('TESTING') is not None:
        break
