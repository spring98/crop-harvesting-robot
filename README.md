# Crop Harvesting Robot

## Preview
https://github.com/spring98/crop-harvesting-robot/assets/92755385/da6ab712-456c-4acd-b27b-8c7d8d3e30c0

## Research Topic
컴퓨터 비전 AI 모델을 활용한 농작물 자동수확 로봇 개발

## Scheme
<img width="709" alt="스크린샷 2024-05-15 12 47 12" src="https://github.com/spring98/crop-harvesting-robot/assets/92755385/ea79acfb-f9d3-4b5a-862a-2d6a6f7f1c85">

## Specification
<img width="725" alt="스크린샷 2024-05-15 12 47 29" src="https://github.com/spring98/crop-harvesting-robot/assets/92755385/e0632018-4db9-442e-b126-3085bf0c9820">

## Kinematics
<img width="721" alt="스크린샷 2024-05-15 12 47 59" src="https://github.com/spring98/crop-harvesting-robot/assets/92755385/b202c9f9-392e-4ec0-90fa-554d5309f04e">

### DH Parameter
| $$i$$  | $$\alpha_{i-1}$$ | $$a_{i-1}$$ | $$d_i$$ | $$\theta_i$$ |
|---|---|---|---|---|
|$$1$$|$$0$$|$$0$$|$$d_1$$|$$\theta_1$$|
|$$2$$|$$-\pi/2$$|$$a_1$$|$$0$$|$$\theta_2$$|
|$$3$$|$$0$$|$$a_2$$|$$0$$|$$\theta_3$$|
|$$4$$|$$\pi/2$$|$$-a_3$$|$$d_4$$|$$\theta_4$$|
|$$5$$|$$-\pi/2$$|$$-a_4$$|$$0$$|$$\theta_5$$|
|$$6$$|$$\pi/2$$|$$a_5$$|$$d_6$$|$$\theta_6$$|

해당 프로젝트에서는 forward kinematics 및 inverse kinematics 에 대한 수식을 따로 유도하지 않았습니다.

이유는 urdf 에 기술된 링크들의 상대적인 위치 및 회전에 대한 정보를 가지고 moveit package 에서 자동으로 관계식을 유도하여 계산하기 때문입니다. 다음으로 이동해야하는 위치 $(x, y, z)$ 를 전달하면 inverse kinematics 의 솔루션 $(\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6)$ 을 제공합니다.

위에서 설계한 매니퓰레이터의 urdf 는 아래와 같이 기술하였습니다.

<details>
<summary> urdf </summary>

```xml
<?xml version='1.0' ?>

<robot name="puma560">

    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>
    <material name="orange">
        <color rgba="1 0.2 0 1"/>
    </material>
    <material name="yellow">
        <color rgba="1 1 0 1"/>
    </material>
    <material name="green">
        <color rgba="0 1 0 1"/>
    </material>
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>
    <material name="black">
        <color rgba="0 0 0 1"/>
    </material>
    <material name="purple">
        <color rgba="0.4 0 1 1"/>
    </material>
    
    <link name="world"/>

    <joint name="joint_world" type="fixed">
        <parent link="world"/>
        <child link="base"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>

    <link name="base">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis0.stl"/>
            </geometry>
            <material name="red"/>
        </visual>
    </link>

    <joint name="joint01" type="revolute">
        <origin xyz="0 0 0.239" rpy="0 0 0"/>
        <parent link="base"/>
        <child link="link1"/>
        <axis xyz='0 0 1'/>
        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-3.14" upper="3.14" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-3.14" soft_upper_limit="3.14" />
    </joint>
    
    <link name="link1">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis1.stl"/>
            </geometry>
            <material name="orange"/>
        </visual>
    </link>

    <joint name="joint12" type="revolute">
        <origin xyz="0.08 0 0.0033" rpy="-1.5707963268 0 0"/>
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz='0 0 1'/>
        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-2.0043950955" upper="1.5707963268" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-3.14" soft_upper_limit="3.14" />
    </joint>

    <link name="link2">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis2.stl"/>
            </geometry>
            <material name="yellow"/>
        </visual>
    </link>

    <joint name="joint23" type="revolute">
        <origin xyz="0.1156 0 -0.002" rpy="0 0 0"/>
        <parent link="link2"/>
        <child link="link3"/>
        <axis xyz='0 0 1'/>
        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-0.3141592653" upper="3.4557519188" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-3.14" soft_upper_limit="3.14" />
    </joint>

    <link name="link3">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis3.stl"/>
            </geometry>
            <material name="green"/>
        </visual>
    </link>

    <joint name="joint34" type="revolute">
        <origin xyz="-0.01234 -0.148 0.00075" rpy="0 1.5707963268 -1.5707963268"/>
        <parent link="link3"/>
        <child link="link4"/>
        <axis xyz='0 0 1'/>
        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-3.14" upper="3.14" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-3.14" soft_upper_limit="3.14" />
    </joint>

    <link name="link4">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis4.stl"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="joint45" type="revolute">
        <origin xyz="-0.0002 -0.0488 0.0602" rpy="-1.5707963268 0 0"/>
        <parent link="link4"/>
        <child link="link5"/>
        <axis xyz='0 0 1'/>
        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-2.0561944901" upper="2.0561944901" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-3.14" soft_upper_limit="3.14" />
    </joint>

    <link name="link5">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis5.stl"/>
            </geometry>
            <material name="black"/>
        </visual>
    </link>

    <joint name="joint56" type="revolute">
        <origin xyz="0 -0.06422 0.04994" rpy="1.5707963268 -1.5707963268 0"/>
        <parent link="link5"/>
        <child link="link6"/>
        <axis xyz='0 0 1'/>
        <calibration rising="0.0"/>
        <dynamics damping="0.0" friction="0.0"/>
        <limit effort="30" velocity="1.0" lower="-3.14" upper="3.14" />
        <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-3.14" soft_upper_limit="3.14" />
    </joint>

    <link name="link6">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
		    <mesh filename="package://puma560/meshes/axis6.stl"/>
            </geometry>
            <material name="purple"/>
        </visual>
    </link>

</robot>

```
</details>

## Trajectory
https://github.com/spring98/crop-harvesting-robot/assets/92755385/9c3eec3f-53af-4a41-8089-c805c6e77ae9

## Dynamics

## Control

## Result
