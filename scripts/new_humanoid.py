from dm_control import mujoco

# Load a model from an MJCF XML string.
xml_string = """
<mujoco model="Humanoid in Soccer Field">
    <!--  Copyright © 2018, Roboti LLC

          This file is licensed under the MuJoCo Resource License (the "License").
          You may not use this file except in compliance with the License.
          You may obtain a copy of the License at

            https://www.roboti.us/resourcelicense.txt
    -->

    <compiler angle="degree"/>

    <option timestep="0.005" iterations="50" solver="Newton" jacobian="sparse" cone="pyramidal" tolerance="1e-10"/>

    <size njmax="1500" nconmax="500" nstack="5000000"/>

    <default>
        <geom solimp=".9 .9 .01"/>       

        <default class="humanoid">
            <geom material="humanoid"/>        
            <joint damping="1" limited="true"/>
            <motor ctrllimited="true" ctrlrange="-.4 .4"/>
        </default>

        <default class="object">
            <geom type="sphere" material="object" size="0.1"/>
        </default>

<!-- Defining Boundaries
        <default class="border">
            <geom type="capsule" size="0.4" rgba=".4 .4 .4 1"/>
        </default>

        <default class="borderpost">
            <geom type="box" size="0.41 0.41 0.41" rgba=".55 .55 .55 1"/>
        </default>
    
-->
   </default>
    <asset>
   <texture builtin="gradient" height="100" rgb1=".4 .5 .6" rgb2="0 0 0" type="skybox" width="100"/>
        <!-- <texture builtin="gradient" height="100" rgb1="1 1 1" rgb2="0 0 0" type="skybox" width="100"/>-->
        <texture builtin="flat" height="1278" mark="cross" markrgb="1 1 1" name="texgeom" random="0.01" rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" type="cube" width="127"/>
        <texture builtin="checker" height="100" name="texplane" rgb1="0 0 0" rgb2="0.8 0.8 0.8" type="2d" width="100"/>
        <material name="MatPlane" reflectance="0.5" shininess="1" specular="1" texrepeat="60 60" texture="texplane"/>
        <material name="geom" texture="texgeom" texuniform="true"/>
        <material name='humanoid' texture="texgeom" texuniform="true" rgba="1.2 1.2 0.6 1"/>
        <material name='object' texture="texgeom" texuniform="true" rgba=".9 .1 .1 1" />
    </asset>

    <visual>
        <quality shadowsize="4096" offsamples="8"/>
        <map znear="0.1" force="0.05"/>
    </visual>

    <statistic extent="4"/>

    <worldbody>

        <light cutoff="100" diffuse="1 1 1" dir="-0 0 -1.3" directional="true" exponent="1" pos="0 0 1.3" specular=".1 .1 .1"/>
	
	<!--Floor-->
        <geom condim="3" friction="1 .1 .1" material="MatPlane" name="floor" pos="0 0 0" rgba="0.8 0.9 0.8 1" size="20 20 0.125" type="plane"/>
	<!--Goal-->
	<geom name="frontpost" type="box" rgba="0 0.8 0.5 1" pos="5.5 0 0" size=".1 1.5 1.5"/>
        <geom name="leftpost" type="box" rgba="0 0.8 0.5 1" pos="5 1.5 0" size="0.7 .1 1.5"/>
        <geom name="rightpost" type="box" rgba="0 0.8 0.5 1"  pos="5 -1.5 0" size="0.7 .1 1.5"/>     
	
	<!--
        <geom class="border" fromto="-3 3 0 3 3 0"  />
        <geom class="border" fromto="-3 -3 0 3 -3 0"  />
        <geom class="border" fromto="3 3 0 3 -3 0"  />
        <geom class="border" fromto="-3 3 0 -3 -3 0"  />
        <geom class="borderpost" pos="3 3 0"/>
        <geom class="borderpost" pos="-3 3 0"/>
        <geom class="borderpost" pos="3 -3 0"/>
        <geom class="borderpost" pos="-3 -3 0"/> 
	-->

        <body name='torso' pos='0 0 1.4' childclass="humanoid">
            <freejoint name="root"/>
            <geom name='torso1' type='capsule' fromto='0 -.07 0 0 .07 0'  size='0.07'/>
            <geom name='head' type='sphere' pos='0 0 .19' size='.09'/>
            <geom name='uwaist' type='capsule' fromto='-.01 -.06 -.12 -.01 .06 -.12' size='0.06'/>
            <body name='lwaist' pos='-.01 0 -0.260' quat='1.000 0 -0.002 0' >
                <geom name='lwaist' type='capsule' fromto='0 -.06 0 0 .06 0'  size='0.06' />
                <joint name='abdomen_z' type='hinge' pos='0 0 0.065' axis='0 0 1' range='-45 45' damping='5' stiffness='20' armature='0.02' />
                <joint name='abdomen_y' type='hinge' pos='0 0 0.065' axis='0 1 0' range='-75 30' damping='5' stiffness='10' armature='0.02' />
                <body name='pelvis' pos='0 0 -0.165' quat='1.000 0 -0.002 0' >
                    <joint name='abdomen_x' type='hinge' pos='0 0 0.1' axis='1 0 0' range='-35 35' damping='5' stiffness='10' armature='0.02' />
                    <geom name='butt' type='capsule' fromto='-.02 -.07 0 -.02 .07 0'  size='0.09' />
                    <body name='right_thigh' pos='0 -0.1 -0.04' >
                        <joint name='right_hip_x' type='hinge' pos='0 0 0' axis='1 0 0' range='-25 5'   damping='5' stiffness='10' armature='0.01' />
                        <joint name='right_hip_z' type='hinge' pos='0 0 0' axis='0 0 1' range='-60 35'  damping='5' stiffness='10' armature='0.01' />
                        <joint name='right_hip_y' type='hinge' pos='0 0 0' axis='0 1 0' range='-120 20' damping='5' stiffness='20' armature='0.01' />
                        <geom name='right_thigh1' type='capsule' fromto='0 0 0 0 0.01 -.34'  size='0.06' />
                        <body name='right_shin' pos='0 0.01 -0.403' >
                            <joint name='right_knee' type='hinge' pos='0 0 .02' axis='0 -1 0' range='-160 -2' stiffness='1' armature='0.0060' />
                            <geom name='right_shin1' type='capsule' fromto='0 0 0 0 0 -.3'   size='0.049' />
                            <body name='right_foot' pos='0 0 -.39' >
                                <joint name='right_ankle_y' type='hinge' pos='0 0 0.08' axis='0 1 0'   range='-50 50' stiffness='4' armature='0.0008' />
                                <joint name='right_ankle_x' type='hinge' pos='0 0 0.04' axis='1 0 0.5' range='-50 50' stiffness='1'  armature='0.0006' />
                                <geom name='right_foot_cap1' type='capsule' fromto='-.07 -0.02 0 0.14 -0.04 0'  size='0.027' />
                                <geom name='right_foot_cap2' type='capsule' fromto='-.07 0 0 0.14  0.02 0'  size='0.027' />
                            </body>
                        </body>
                    </body>
                    <body name='left_thigh' pos='0 0.1 -0.04' >
                        <joint name='left_hip_x' type='hinge' pos='0 0 0' axis='-1 0 0' range='-25 5'  damping='5' stiffness='10' armature='0.01' />
                        <joint name='left_hip_z' type='hinge' pos='0 0 0' axis='0 0 -1' range='-60 35' damping='5' stiffness='10' armature='0.01' />
                        <joint name='left_hip_y' type='hinge' pos='0 0 0' axis='0 1 0' range='-120 20' damping='5' stiffness='20' armature='0.01' />
                        <geom name='left_thigh1' type='capsule' fromto='0 0 0 0 -0.01 -.34'  size='0.06' />
                        <body name='left_shin' pos='0 -0.01 -0.403' >
                            <joint name='left_knee' type='hinge' pos='0 0 .02' axis='0 -1 0' range='-160 -2' stiffness='1' armature='0.0060' />
                            <geom name='left_shin1' type='capsule' fromto='0 0 0 0 0 -.3'   size='0.049' />
                            <body name='left_foot' pos='0 0 -.39' >
                                <joint name='left_ankle_y' type='hinge' pos='0 0 0.08' axis='0 1 0'   range='-50 50'  stiffness='4' armature='0.0008' />
                                <joint name='left_ankle_x' type='hinge' pos='0 0 0.04' axis='1 0 0.5' range='-50 50'  stiffness='1'  armature='0.0006' />
                                <geom name='left_foot_cap1' type='capsule' fromto='-.07 0.02 0 0.14 0.04 0'  size='0.027' />
                                <geom name='left_foot_cap2' type='capsule' fromto='-.07 0 0 0.14  -0.02 0'  size='0.027' />
                            </body>
                        </body>
                    </body>
                </body>
            </body>
            <body name='right_upper_arm' pos='0 -0.17 0.06' >
                <joint name='right_shoulder1' type='hinge' pos='0 0 0' axis='2 1 1'  range='-85 60' stiffness='1' armature='0.0068' />
                <joint name='right_shoulder2' type='hinge' pos='0 0 0' axis='0 -1 1' range='-85 60' stiffness='1'  armature='0.0051' />
                <geom name='right_uarm1' type='capsule' fromto='0 0 0 .16 -.16 -.16'  size='0.04 0.16' />
                <body name='right_lower_arm' pos='.18 -.18 -.18' >
                    <joint name='right_elbow' type='hinge' pos='0 0 0' axis='0 -1 1' range='-90 50'  stiffness='0' armature='0.0028' />
                    <geom name='right_larm' type='capsule' fromto='0.01 0.01 0.01 .17 .17 .17'  size='0.031' />
                    <geom name='right_hand' type='sphere' pos='.18 .18 .18'  size='0.04'/>
                </body>
            </body>
            <body name='left_upper_arm' pos='0 0.17 0.06' >
                <joint name='left_shoulder1' type='hinge' pos='0 0 0' axis='2 -1 1' range='-60 85' stiffness='1' armature='0.0068' />
                <joint name='left_shoulder2' type='hinge' pos='0 0 0' axis='0 1 1' range='-60 85'  stiffness='1' armature='0.0051' />
                <geom name='left_uarm1' type='capsule' fromto='0 0 0 .16 .16 -.16'  size='0.04 0.16' />
                <body name='left_lower_arm' pos='.18 .18 -.18' >
                    <joint name='left_elbow' type='hinge' pos='0 0 0' axis='0 -1 -1' range='-90 50' stiffness='0' armature='0.0028' />
                    <geom name='left_larm' type='capsule' fromto='0.01 -0.01 0.01 .17 -.17 .17'  size='0.031' />
                    <geom name='left_hand' type='sphere' pos='.18 -.18 .18'  size='0.04'/>
                </body>
            </body>
        </body>

        <body pos="1 0.5 0" quat="0.471405 0.471405 0.707107 0.235702">
            <freejoint/>
            <geom class="object"/>
        </body>
    </worldbody>

    <tendon>
        <fixed name="left_hipknee">
            <joint coef="-1" joint="left_hip_y"/>
            <joint coef="1" joint="left_knee"/>
        </fixed>
        <fixed name="right_hipknee">
            <joint coef="-1" joint="right_hip_y"/>
            <joint coef="1" joint="right_knee"/>
        </fixed>
    </tendon>

    <actuator>
        <motor gear="100" joint="abdomen_y" name="abdomen_y"/>
        <motor gear="100" joint="abdomen_z" name="abdomen_z"/>
        <motor gear="100" joint="abdomen_x" name="abdomen_x"/>
        <motor gear="100" joint="right_hip_x" name="right_hip_x"/>
        <motor gear="100" joint="right_hip_z" name="right_hip_z"/>
        <motor gear="300" joint="right_hip_y" name="right_hip_y"/>
        <motor gear="200" joint="right_knee" name="right_knee"/>
        <motor gear="100" joint="left_hip_x" name="left_hip_x"/>
        <motor gear="100" joint="left_hip_z" name="left_hip_z"/>
        <motor gear="300" joint="left_hip_y" name="left_hip_y"/>
        <motor gear="200" joint="left_knee" name="left_knee"/>
        <motor gear="25" joint="right_shoulder1" name="right_shoulder1"/>
        <motor gear="25" joint="right_shoulder2" name="right_shoulder2"/>
        <motor gear="25" joint="right_elbow" name="right_elbow"/>
        <motor gear="25" joint="left_shoulder1" name="left_shoulder1"/>
        <motor gear="25" joint="left_shoulder2" name="left_shoulder2"/>
        <motor gear="25" joint="left_elbow" name="left_elbow"/>
    </actuator>



</mujoco>
"""
physics = mujoco.Physics.from_xml_string(xml_string)

# Render the default camera view as a numpy array of pixels.
pixels = physics.render()

# Reset the simulation, move the slide joint upwards and recompute derived
# quantities (e.g. the positions of the body and geoms).

# Print the positions of the geoms.
print(physics.named.data.geom_xpos)
# FieldIndexer(geom_xpos):
#            x         y         z
# 0  floor [ 0         0         0       ]
# 1    box [ 0         0         0.8     ]
# 2 sphere [ 0.2       0.2       1       ]

# Advance the simulation for 1 second.
while physics.time() < 1.:
  physics.step()

# Print the new z-positions of the 'box' and 'sphere' geoms.
#print(physics.named.data.geom_xpos[['box', 'object'], 'x'])
# [ 0.19996362  0.39996362]
