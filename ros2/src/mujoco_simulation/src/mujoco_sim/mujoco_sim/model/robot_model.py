import os
from ament_index_python.packages import get_package_share_directory

class XMLModel:
    """ Class to load and manipulate MuJoCo XML model files dynamically by
    changing the model attributes, and returning the modified XML model file as string.
    """

    def __init__(self) -> None:
        self.exoskeleton_str = None
        self.no_safety_catchers = 0

        self.obstacle = None
        self.obstacle_str = ""

        self.safety_catch_x = False
        self.safety_catch_y = False
        self.safety_catch_z = False
        self.safety_catch_roll = False
        self.safety_catch_pitch = False
        self.safety_catch_yaw = False

    def build_mujoco_model(self) -> str:
        prefix_mujoco_str = "<mujoco model='exoskeleton'>"
        suffix_mujoco_str = "</mujoco>"

        return prefix_mujoco_str + \
            self.configure_model() + \
            self.build_assets() + \
            self.build_defaults() + \
            self.build_world() + \
            self.build_actuators() + \
            self.build_sensors() + \
            self.build_keyframes() + \
            suffix_mujoco_str

    def configure_model(self) -> str:
        config_str = """
            <compiler meshdir="obj-files" angle="degree" autolimits="true"/>
            <size nuser_actuator='1' nuser_sensor='1' nuser_geom='1'/>
            <option timestep='0.01' iterations='50' solver='PGS' gravity='0 0 -9.81'>
                <!-- <0flag energy="enable"/> -->
            </option>

            <visual>
                <map force="0.1" zfar="30"/>
                <rgba haze="0.15 0.25 0.35 1"/>
                <quality shadowsize="4096"/>
                <global offwidth="800" offheight="800"/>
            </visual>
            """
        return config_str
    
    def build_assets(self) -> str:
        mesh_dir = get_package_share_directory('march_description') + '/urdf/march9/obj-files'
        return f"""
            <asset>
                <texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="512" height="512"/>
                <texture name="body" type="cube" builtin="flat" mark="cross" width="127" height="1278"
                        rgb1="0.8 0.6 0.4" rgb2="0.8 0.6 0.4" markrgb="1 1 1" random="0.01"/>
                <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
                <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
                <material name="march_grey" rgba=".87 .89 .91 1"/>
                <material name="collision" rgba="1 0 0 1"/>

                <mesh name="backpack"    file="{mesh_dir}/Backpack.obj"/>
                <mesh name="L_Ankle"     file="{mesh_dir}/L_Ankle.obj"/>
                <mesh name="L_Hip"       file="{mesh_dir}/L_Hip.obj"/>
                <mesh name="L_UpperLeg"  file="{mesh_dir}/L_UpperLeg.obj"/>
                <mesh name="L_LowerLeg"  file="{mesh_dir}/L_LowerLeg.obj"/>
                <mesh name="L_Sole"      file="{mesh_dir}/L_Sole.obj"/>

                <mesh name="R_Ankle"     scale="1 -1 1"    file="{mesh_dir}/L_Ankle.obj"/>
                <mesh name="R_Hip"       scale="1 -1 1"    file="{mesh_dir}/L_Hip.obj"/>
                <mesh name="R_UpperLeg"  scale="1 -1 1"    file="{mesh_dir}/L_UpperLeg.obj"/>
                <mesh name="R_LowerLeg"  scale="1 -1 1"    file="{mesh_dir}/L_LowerLeg.obj"/>
                <mesh name="R_Sole"      scale="1 -1 1"    file="{mesh_dir}/L_Sole.obj"/>
            </asset>
        """
    
    def build_defaults(self) -> str:
        return """
            <default>
                <site group="5"/>
                <motor ctrlrange="-1 1" ctrllimited="true"/>
                <geom contype="0" conaffinity="0" condim="1" solref="0.005 1"/>
                <equality solref="0.005 1"/>
                <default class="exoskeleton">
                <geom material="march_grey" group="2"/>
                </default>
                <default class="collision">
                <geom contype="1" group="3" type="capsule"/>
                <default class="L_collision">
                    <geom contype="2" conaffinity="4"/>
                </default>
                <default class="R_collision">
                    <geom contype="4" conaffinity="2"/>
                </default>
                </default>
                <default class="HAA_joint">
                <joint damping="107.692308"
                        armature="5.4"
                        frictionloss="0" stiffness="0"/>
                </default>
                <default class="ADPF_joint">
                <joint damping="43.658537"
                        armature="2.2"
                        frictionloss="0" stiffness="0"/>
                </default>
                <default class="rotational_joint">
                <joint damping="63.451777"
                        armature="9.98"
                        frictionloss="0" stiffness="0"/>
                </default>
            </default>
            <default class="AIE_joint">
                <joint damping="0"
                        armature="0"
                    frictionloss="0" stiffness="0"/>
            </default>
        """

    def build_world(self) -> str:
        prefix_worldbody_str = "<worldbody>"
        suffix_worldbody_str = "</worldbody>"

        floor_str = "<geom name='floor' size='0 0 .05' type='plane' material='grid' conaffinity='15' condim='3'/>"
        spotlight_str = "<light name='spotlight' mode='targetbodycom' target='backpack' diffuse='.8 .8 .8' specular='0.3 0.3 0.3' pos='0 -20 4' cutoff='10'/>"

        return prefix_worldbody_str + floor_str + spotlight_str + self.obstacle_str + self.exoskeleton_str + suffix_worldbody_str

    def build_exoskeleton(self, x: bool, y: bool, z: bool, roll: bool, pitch: bool, yaw: bool) -> None:
        # Determine if safety catchers are needed.
        safety_catcher_str = ""
        if x:
            safety_catcher_str = safety_catcher_str + "<joint name='safety_catch_slider_x' type='slide' axis='1 0 0' range='-100 100' pos='0 0 0'/>"
            self.no_safety_catchers += 1
            self.safety_catch_x = True
        if y:
            safety_catcher_str = safety_catcher_str + "<joint name='safety_catch_slider_y' type='slide' axis='0 1 0' range='-100 100' pos='0 0 0'/>"
            self.no_safety_catchers += 1
            self.safety_catch_y = True
        if z:
            safety_catcher_str = safety_catcher_str + "<joint name='safety_catch_slider_z' type='slide' axis='0 0 1' range='-100 100' pos='0 0 0'/>"
            self.no_safety_catchers += 1
            self.safety_catch_z = True
        if roll:
            safety_catcher_str = safety_catcher_str + "<joint name='safety_catch_hinge_x' type='hinge' axis='1 0 0' range='-8 8' pos='0 0 0'/>"
            self.no_safety_catchers += 1
            self.safety_catch_roll = True
        if pitch:
            safety_catcher_str = safety_catcher_str + "<joint name='safety_catch_hinge_y' type='hinge' axis='0 1 0' range='-180 180' pos='0 0 0'/>"
            self.no_safety_catchers += 1
            self.safety_catch_pitch = True
        if yaw:
            safety_catcher_str = safety_catcher_str + "<joint name='safety_catch_hinge_z' type='hinge' axis='0 0 1' range='-180 180' pos='0 0 0'/>"
            self.no_safety_catchers += 1
            self.safety_catch_yaw = True

        self.exoskeleton_str = f"""
            <body name="safety_catcher" pos="-0.2 0 1.2">
            <inertial pos="0 0 0" mass="0.1" diaginertia="10 10 10"/>

            {safety_catcher_str}
                   
            <body name="backpack" pos="0 0 0" childclass="exoskeleton">
                <light name="top" pos="0 0 2" mode="trackcom"/>
                <camera name="diag" pos="3 -2.5 1" xyaxes="1 1 0 0 1 2" mode="track"/>
                <camera name="right" pos="0 -3 1" xyaxes="1 0 0 0 1 2" mode="track"/>
                <site name='imu_backpack' size='0.01' pos="0 0 0"/>

                <inertial pos="0.015 0 0.194" mass="3.362"
                        fullinertia="0.1373 0.0379 0.1005 0.000 -0.0002 0.000"/>
                <!-- <freejoint/> -->
                <geom type="mesh" mesh="backpack" />
                <geom name="backpack" type="box"
                    size="0.051 0.189 0.165"
                    pos="0 0 0.17" class="collision"/>

                <site name="imu_torso" size='0.01' pos="0.2 0 0.4"/>

                <!--  LEFT LEG -->
                <body name="L_hip" pos="0.021 0.092 0.028">
                <inertial 
                    mass="3.684"
                    pos="0.130 0.174 0.000" 
                    fullinertia="0.0146 0.0255 0.0321 -0.0106 0.0000 0.000"/>
                <joint name="left_hip_aa" axis="-1 0 0" range="-20 10" class="HAA_joint" />
                <geom type="mesh" mesh="L_Hip" />
                <geom name="L_hip" fromto="0 0 0 0.169 0.173 0"
                    size="0.032" class="L_collision"/>
                <site name="tor_L_HAA" size="0.01" pos="0 0 0"/>

                <body name="L_UL" pos="0.169 0.173 0">
                    <inertial 
                    mass="1.520"
                    pos="0.000 -0.044 -0.408" 
                    fullinertia="0.0816 0.0831 0.0069 0.0000 0.0000 -0.0115"/>
                    <joint name="left_hip_fe" axis="0 -1 0" range="-20 115" class="rotational_joint" />
                    <geom type="mesh" mesh="L_UpperLeg"/>
                    <geom name="L_UL" fromto="0 0 0 0 -0.077 -0.478"
                    size="0.069" class="L_collision"/>
                    <site name="tor_L_HFE" size="0.01" pos="0 0 0"/>

                    <body name="L_LL" pos="0 -0.077 -0.478">
                    <inertial 
                        mass="1.895"
                        pos="-0.049 0.000 -0.269" 
                        fullinertia="0.0388 0.0427 0.0046 -0.0005 -0.0049 -0.0015"/>
                    <joint name="left_knee" axis="0 1 0" range="0 125" class="rotational_joint" />
                    <geom type="mesh" mesh="L_LowerLeg"/>
                    <site name="tor_L_KFE" size="0.01" pos="0 0 0"/>
                    <geom name="L_LL" fromto="0 0 0 0.000 -0.016 -0.474"
                        size="0.047" class="L_collision"/>

                    <body name="L_ankle" pos="0.000 -0.016 -0.474">
                        <inertial 
                        mass="0.683"
                        pos="0.070 0.043 -0.002" 
                        fullinertia="0.0011 0.0043 0.0047 -0.0010 -0.0006 -0.0003"/>
                        <joint name="left_ankle_dpf" axis="0 -1 0" range="-28 20" class="ADPF_joint"/>
                        <geom type="mesh" mesh="L_Ankle"/>
                        <site name="tor_L_ADPF" size="0.01" pos="0 0 0"/>

                        <body name="L_foot" pos="-0.093 -0.061 -0.025">
                        <inertial
                            mass="0.589" 
                            pos="0.139 0.001 -0.052" 
                            fullinertia="0.0016 0.0077 0.0083 0.000 -0.0001 0.000"/>
                        <joint name="left_ankle_ie" axis="1 0 0" range="-9 9" class="AIE_joint"/>
                        <geom type="mesh" mesh="L_Sole"/>
                        <site name="tor_L_AIE" size="0.01" pos="0 0 0"/>

                        <geom name="L_foot" type="box" size="0.166 0.07 0.035"
                            pos="0.151 0 -.045"  class="L_collision"
                            friction="3.0"/>

                        </body>
                    </body>
                    </body>
                </body>
                </body>

                <!--  RIGHT LEG -->
                <body name="R_hip" pos="0.021 -0.092 0.028">
                <inertial 
                    mass="3.684"
                    pos="0.130 -0.174 0.000" 
                    fullinertia="0.0146 0.0255 0.0321 0.0106 0.000 0.000"/>
                <joint name="right_hip_aa" axis="1 0 0" range="-20 10" class="HAA_joint" />
                <geom type="mesh" mesh="R_Hip" />
                <geom name="R_hip" fromto="0 0 0 0.169 -0.173 0"
                    size="0.032" class="R_collision"/>
                <site name="tor_R_HAA" size="0.01" pos="0 0 0"/>

                <body name="R_UL" pos="0.169 -0.173 0">
                    <inertial 
                    mass="1.520"
                    pos="0.000 0.044 -0.408"
                    fullinertia="0.0816 0.0831 0.0069 0.0000 0.0000 0.0115"/>
                    <joint name="right_hip_fe" axis="0 -1 0" range="-20 115" class="rotational_joint" />
                    <geom type="mesh" mesh="R_UpperLeg"/>
                    <geom name="R_UL" fromto="0 0 0 0 0.077 -0.478"
                    size="0.069" class="R_collision"/>
                    <site name="tor_R_HFE" size="0.01" pos="0 0 0"/>

                    <body name="R_LL" pos="0 0.077 -0.478">
                    <inertial 
                        mass="1.895"
                        pos="-0.049 0.000 -0.269"
                        fullinertia="0.0388 0.0427 0.0046 0.0005 -0.0049 0.0015"/>
                    <joint name="right_knee" axis="0 1 0" range="0 125" class="rotational_joint" />
                    <geom type="mesh" mesh="R_LowerLeg"/>
                    <geom name="R_LL" fromto="0 0 0 0.000 0.016 -0.474"
                        size="0.047" class="R_collision"/>
                    <site name="tor_R_KFE" size="0.01" pos="0 0 0"/>

                    <body name="R_ankle" pos="0.000 0.016 -0.474">
                        <inertial 
                        mass="0.683"
                        pos="0.070 -0.043 -0.002"
                        fullinertia="0.0011 0.0043 0.0047 0.0010 -0.0006 0.0003"/>
                        <joint name="right_ankle_dpf" axis="0 -1 0" range="-28 20" class="ADPF_joint"/>
                        <geom type="mesh" mesh="R_Ankle"/>
                        <site name="tor_R_ADPF" size="0.01" pos="0 0 0"/>

                        <body name="R_foot" pos="-0.093 0.061 -0.025">
                        <inertial 
                            mass="0.589"
                            pos="0.139 -0.001 -0.052" 
                            fullinertia="0.0016 0.0077 0.0083 0.000 -0.0001 0.000"/>
                        <joint name="right_ankle_ie" axis="-1 0 0" range="-9 9" class="AIE_joint"/>
                        <geom type="mesh" mesh="R_Sole"/>
                        <site name="tor_R_AIE" size="0.01" pos="0 0 0"/>

                        <geom name="R_foot" type="box" size="0.166 0.07 0.035"
                            pos="0.151 0 -.045" class="R_collision"
                            friction="3.0"/>

                        </body>
                    </body>
                    </body>
                </body>
                </body>
            </body>
            </body>
        """

    def build_obstacle(self, obstacle=None) -> None:
        self.obstacle = obstacle
        starting_line_str = """
            <body name="starting_line" pos="0 0 0">
                <geom type="box" size="0.05 1.5 1e-3" rgba="1 1 1 1"/>
            </body>
        """

        finish_line_str = """
                <body name="finish_line" pos="5.0 0 0">
                    <geom type="box" size="0.05 1.5 1e-3" rgba="1 1 1 1"/>
                </body>
            """

        if obstacle == "sit":
            self.obstacle_str = """
                    <body name="box" pos="-0.1 0 0.25">
                        <geom type="box" size="0.25 0.25 0.25" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                """
        elif obstacle == "high-step":
            distance_between_boxes = 1.4
            starting_distance = 0.69
            self.obstacle_str = f"""
                    <body name="high_step_box_1" pos="{starting_distance} 0 0.09">
                        <geom type="box" size="0.150 0.6 0.09" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="high_step_box_2" pos="{starting_distance + distance_between_boxes} 0 0.09">
                        <geom type="box" size="0.145 0.6 0.11" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="high_step_box_3" pos="{starting_distance + 2.0 * distance_between_boxes} 0 0.09">
                        <geom type="box" size="0.165 0.6 0.13" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    {starting_line_str}
                    {finish_line_str}
                """
        elif obstacle == "stairs":
            starting_distance = 2.525
            self.obstacle_str = f"""
                <body name="stair_ledge_1" pos="{starting_distance} 0 0.0855">
                    <geom type="box" size="1.445 1.25 0.0855" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    <body name="stair_ledge_2" pos="-0.07 0 0.171">
                        <geom type="box" size="1.13 1.25 0.0855" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                        <body name="stair_ledge_3" pos="-0.07 0 0.171">
                            <geom type="box" size="0.815 1.25 0.0855" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                            <body name="stair_ledge_4" pos="-0.07 0 0.171">
                                <geom type="box" size="0.5 1.25 0.0855" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                            </body>
                        </body>
                    </body>
                </body>
                {starting_line_str}
                {finish_line_str}
                """
        elif obstacle == "train":
            self.obstacle_str = f"""
                <body name="train_bench_1" pos="1.97 0 0.42">
                    <geom type="box" size="0.25 0.53 0.015" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    <body name="train_bench_1_leg_1" pos="0.25 0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_1_leg_2" pos="-0.25 0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_1_leg_3" pos="0.25 -0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_1_leg_4" pos="-0.25 -0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_1_back" pos="-0.25 0 0.18" euler="0 -5 0">
                        <geom type="box" size="0.015 0.53 0.18" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                </body>
                <body name="train_bench_2" pos="3.08 0 0.42" euler="0 0 180">
                    <geom type="box" size="0.25 0.53 0.015" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    <body name="train_bench_2_leg_1" pos="0.25 0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_2_leg_2" pos="-0.25 0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_2_leg_3" pos="0.25 -0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_2_leg_4" pos="-0.25 -0.53 -0.21">
                        <geom type="box" size="0.015 0.015 0.21" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                    <body name="train_bench_2_back" pos="-0.25 0 0.18" euler="0 -5 0">
                        <geom type="box" size="0.015 0.53 0.18" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    </body>
                </body>
                {starting_line_str}
                {finish_line_str}
            """
        elif obstacle == "tilt":
            self.obstacle_str = f"""
                <body name="ramp" pos="2.5 1.525 0.15" euler="12 0 0">
                    <geom type="box" size="1.526 1.51 0.1687" rgba="0.8 0.6 0.4 1" conaffinity="15" condim="3"/>
                    <body name="ramp_tape" pos="0 -1.51 0.075">
                        <geom type="box" size="0.5 0.3 0.1" rgba="1 0 0 1"/>
                    </body>
                </body>
                <body name="red_tape" pos="2.5 -0.75 0.001">
                    <geom type="box" size="0.5 0.75 0.001" rgba="1 0 0 1"/>
                </body>
                {starting_line_str}
                {finish_line_str}
            """
        else:
            self.obstacle_str = ""

    def build_actuators(self) -> str:
        return """
            <actuator>
                <!-- User parameter is the maximum no-load motor RPM -->
                <motor name="left_ankle_dpf"    gear="179"  joint="left_ankle_dpf" forcerange="-3.8 3.8"/>
                <motor name="left_hip_aa"     gear="280"  joint="left_hip_aa" forcerange="-3.8 3.8"/>
                <motor name="left_hip_fe"     gear="250" joint="left_hip_fe" forcerange="-3.8 3.8"/>
                <motor name="left_knee"     gear="250"  joint="left_knee" forcerange="-3.8 3.8"/>
                <motor name="right_ankle_dpf"    gear="179"  joint="right_ankle_dpf" forcerange="-3.8 3.8"/>
                <motor name="right_hip_aa"     gear="280"  joint="right_hip_aa" forcerange="-3.8 3.8"/>
                <motor name="right_hip_fe"     gear="250" joint="right_hip_fe" forcerange="-3.8 3.8"/>
                <motor name="right_knee"     gear="250"  joint="right_knee" forcerange="-3.8 3.8"/>
            </actuator>
            """
    
    def build_sensors(self) -> str:
        # return """
        #     <sensor>
        #         <!-- User parameter is the number of absolute encoder bits -->
        #         <!-- encodersoutput position; connected to the actuator -->
        #         <jointpos name="L_ADPF_pos_output" joint="left_ankle_dpf" user="18" noise="000"/>
        #         <jointpos name="L_AIE_pos_output" joint="left_ankle_ie" user="18" noise="000"/>
        #         <jointpos name="L_HAA_pos_output" joint="left_hip_aa" user="18" noise="000"/>
        #         <jointpos name="L_HFE_pos_output" joint="left_hip_fe" user="18" noise="000"/>
        #         <jointpos name="L_KFE_pos_output" joint="left_knee" user="18" noise="000"/>
        #         <jointpos name="R_ADPF_output" joint="right_ankle_dpf" user="18" noise="000"/>
        #         <jointpos name="R_AIE_output" joint="right_ankle_ie" user="18" noise="000"/>
        #         <jointpos name="R_HAA_output" joint="right_hip_aa" user="18" noise="000"/>
        #         <jointpos name="R_HFE_output" joint="right_hip_fe" user="18" noise="000"/>
        #         <jointpos name="R_KFE_output" joint="right_knee" user="18" noise="000"/>
        #         <!-- encodersoutput velocity; connected to the actuator -->
        #         <jointvel name="L_ADPF_vel_output" joint="left_ankle_dpf" user="18" noise="000"/>
        #         <jointvel name="L_AIE_vel_output" joint="left_ankle_ie" user="18" noise="000"/>
        #         <jointvel name="L_HAA_vel_output" joint="left_hip_aa" user="18" noise="000"/>
        #         <jointvel name="L_HFE_vel_output" joint="left_hip_fe" user="18" noise="000"/>
        #         <jointvel name="L_KFE_vel_output" joint="left_knee" user="18" noise="000"/>
        #         <jointvel name="R_ADPF_vel_output" joint="right_ankle_dpf" user="18" noise="000"/>
        #         <jointvel name="R_AIE_vel_output" joint="right_ankle_ie" user="18" noise="000"/>
        #         <jointvel name="R_HAA_vel_output" joint="right_hip_aa" user="18" noise="000"/>
        #         <jointvel name="R_HFE_vel_output" joint="right_hip_fe" user="18" noise="000"/>
        #         <jointvel name="R_KFE_vel_output" joint="right_knee" user="18" noise="000"/>
        #         <!-- torque sensors; connected to the actuator-->

        #     <!--    TODO: This could also be of the type actuatorvel -->
        #         <torque name="L_ADPF_tor_output" site="tor_L_ADPF" user="13" noise="000"/>
        #         <torque name="L_AIE_tor_output" site="tor_L_AIE" user="13" noise="000"/>
        #         <torque name="L_HAA_tor_output" site="tor_L_HAA" user="13" noise="000"/>
        #         <torque name="L_HFE_tor_output" site="tor_L_HFE" user="13" noise="000"/>
        #         <torque name="L_KFE_tor_output" site="tor_L_KFE" user="13" noise="000"/>
        #         <torque name="R_ADPF_tor_output" site="tor_R_ADPF" user="13" noise="000"/>
        #         <torque name="R_AIE_tor_output" site="tor_R_AIE" user="13" noise="000"/>
        #         <torque name="R_HAA_tor_output" site="tor_R_HAA" user="13" noise="000"/>
        #         <torque name="R_HFE_tor_output" site="tor_R_HFE" user="13" noise="000"/>
        #         <torque name="R_KFE_tor_output" site="tor_R_KFE" user="13" noise="000"/>

        #         <!-- Noise & cutoff must be determined -->
        #         <framepos name='backpack_position' objtype='site' objname='imu_backpack'/>
        #         <framelinvel name='backpack_velocity' objtype='site' objname='imu_backpack'/>
        #         <framequat name='backpack_orientation' objtype='site' objname='imu_backpack'/>
        #         <gyro name='vel_imu_backpack' site='imu_backpack' noise='5e-4' cutoff='34.9'/>
        #         <accelerometer name='acc_imu_backpack' site='imu_backpack' noise='1e-5' cutoff='157'/>
        #         <magnetometer name='magnet_imu_backpack' site='imu_backpack'/>

        #         <framequat name='torso_orientation' objtype='site' objname='imu_torso'/>
        #         <gyro name='vel_imu_torso' site='imu_torso' noise='5e-4' cutoff='34.9'/>
        #         <accelerometer name='acc_imu_torso' site='imu_torso' noise='1e-5' cutoff='157'/>
        #         <magnetometer name='magnet_imu_torso' site='imu_torso'/>
        #     </sensor>
        #     """
        return """
            <sensor>
                <!-- User parameter is the number of absolute encoder bits -->
                <!-- encodersoutput position; connected to the actuator -->
                <jointpos name="L_ADPF_pos_output" joint="left_ankle_dpf" user="18" noise="000"/>
                <jointpos name="L_AIE_pos_output" joint="left_ankle_ie" user="18" noise="000"/>
                <jointpos name="L_HAA_pos_output" joint="left_hip_aa" user="18" noise="000"/>
                <jointpos name="L_HFE_pos_output" joint="left_hip_fe" user="18" noise="000"/>
                <jointpos name="L_KFE_pos_output" joint="left_knee" user="18" noise="000"/>
                <jointpos name="R_ADPF_output" joint="right_ankle_dpf" user="18" noise="000"/>
                <jointpos name="R_AIE_output" joint="right_ankle_ie" user="18" noise="000"/>
                <jointpos name="R_HAA_output" joint="right_hip_aa" user="18" noise="000"/>
                <jointpos name="R_HFE_output" joint="right_hip_fe" user="18" noise="000"/>
                <jointpos name="R_KFE_output" joint="right_knee" user="18" noise="000"/>
                <!-- encodersoutput velocity; connected to the actuator -->
                <jointvel name="left_ankle_dpf_vel_output" joint="left_ankle_dpf" user="18" noise="000"/>
                <jointvel name="left_hip_aa_vel_output" joint="left_hip_aa" user="18" noise="000"/>
                <jointvel name="left_hip_fe_vel_output" joint="left_hip_fe" user="18" noise="000"/>
                <jointvel name="left_knee_vel_output" joint="left_knee" user="18" noise="000"/>
                <jointvel name="right_ankle_dpf_vel_output" joint="right_ankle_dpf" user="18" noise="000"/>
                <jointvel name="right_hip_aa_vel_output" joint="right_hip_aa" user="18" noise="000"/>
                <jointvel name="right_hip_fe_vel_output" joint="right_hip_fe" user="18" noise="000"/>
                <jointvel name="right_knee_vel_output" joint="right_knee" user="18" noise="000"/>
                <!-- torque sensors; connected to the actuator-->

            <!--    TODO: This could also be of the type actuatorvel -->
                <torque name="left_ankle_dpf_tor_output" site="tor_L_ADPF" user="13" noise="000"/>
                <torque name="left_hip_aa_tor_output" site="tor_L_HAA" user="13" noise="000"/>
                <torque name="right_ankle_dpf_tor_output" site="tor_R_ADPF" user="13" noise="000"/>
                <torque name="right_hip_aa_tor_output" site="tor_R_HAA" user="13" noise="000"/>

                <!-- Noise & cutoff must be determined -->
                <framepos name='backpack_position' objtype='site' objname='imu_backpack'/>
                <framelinvel name='backpack_velocity' objtype='site' objname='imu_backpack'/>
                <framequat name='backpack_orientation' objtype='site' objname='imu_backpack'/>
                <gyro name='vel_imu_backpack' site='imu_backpack' noise='5e-4' cutoff='34.9'/>
                <accelerometer name='acc_imu_backpack' site='imu_backpack' noise='1e-5' cutoff='157'/>
                <magnetometer name='magnet_imu_backpack' site='imu_backpack'/>

                <framequat name='torso_orientation' objtype='site' objname='imu_torso'/>
                <gyro name='vel_imu_torso' site='imu_torso' noise='5e-4' cutoff='34.9'/>
                <accelerometer name='acc_imu_torso' site='imu_torso' noise='1e-5' cutoff='157'/>
                <magnetometer name='magnet_imu_torso' site='imu_torso'/>
            </sensor>
            """
    
    def build_keyframes(self) -> str:
        safety_catcher_qpos_str = ""
        x = 0.0
        y = 0.0
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        prefix_keyframes_str = "<keyframe>"
        suffix_keyframes_str = "</keyframe>"

        if self.obstacle == "sit":
            z = -0.696
            keyframes_str = f"<key name='sit' qpos='{self.generate_qpos_str(x, y, z, roll, pitch, yaw)} 0 1.57 1.57 0 0 0 1.57 1.57 0 0'/>"
        elif self.obstacle in ["train", "tilt"]:
            y = -1.0
            keyframes_str = f"<key name='train' qpos='{self.generate_qpos_str(x, y, z, roll, pitch, yaw)} 0 0 0 0 0 0 0 0 0 0'/>"
        else:
            keyframes_str = f"<key name='stand' qpos='{self.generate_qpos_str(x, y, z, roll, pitch, yaw)} -0.082 0 0 0 0 -0.082 0 0 0 0'/>"

        return prefix_keyframes_str + keyframes_str + suffix_keyframes_str

    def generate_qpos_str(self, x, y, z, roll, pitch, yaw) -> str:
        qpos = []
        if self.safety_catch_x:
            qpos.append(str(x))
        if self.safety_catch_y:
            qpos.append(str(y))
        if self.safety_catch_z:
            qpos.append(str(z))
        if self.safety_catch_roll:
            qpos.append(str(roll))
        if self.safety_catch_pitch:
            qpos.append(str(pitch))
        if self.safety_catch_yaw:
            qpos.append(str(yaw))
        return " ".join(qpos)
    
    def get_camera_attributes(self) -> dict:
        camera_attributes = {
            "type": 1,
            "distance": 3.0,
            "azimuth": 140.0,
            "elevation": 2,
            "lookat": [0, 0, 0.1]
        }

        if self.obstacle in ["train", "tilt", "high-step", "stairs"]:
            camera_attributes["distance"] = 4.0
            camera_attributes["elevation"] = -10.0
            camera_attributes["azimuth"] = 90.0
        
        return camera_attributes