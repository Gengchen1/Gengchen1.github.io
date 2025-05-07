---
title: ROS2 Launch脚本编写
date: 2024-11-27
summary: Launch脚本编写
category: ROS2
tags:
  - '#Launch脚本'
comments: true
---

> ros2中 launch启动脚本的方式有三种：.xml, .yaml, .launch.py， 本文讲一下 `.launch.py`文件的编写规则。

Launch 三大组件：动作、条件和替换

- 动作：除了可以是一个节点外，还可以是一句打印，一段终端指令，或者另外一个luanch文件
- 替换(参数)：使用launch的参数替换节点的参数值
- 条件：利用条件可以决定哪些动作启动，哪些不启动，类型 `if`

1. 引入需要的基础头文件

```python
import launch
import launch_ros
```

2. 主要的函数，`generate_launch_description()`， `launch` 命令主要识别该函数来执行脚本

```python
def gengerate_launch_description():
	""" 产生launch描述 """
	# 编写启动节点动作
	action_node_turtlesim_node = launch_ros.actions.Node(
		package='turtlesim',
		execultable='turtlesim_node',
		output='screen'	#将节点的输出打印信息只打印在屏幕上
	)
	action_node_partol_client = launch_ros.actions.Node(
		package='demo_cpp_service',
		execultable='partol_client',
		output='log'，
	)
	action_node_turtle_control = launch_ros.actions.Node(
		package='demo_cpp_service',
		execultable='turtle_control',
		output='both',
	)

	return launch.LaunchDescription([
		# 动作 相当于 ros2 run package execultable
		action_node_turtlesim_node,
		action_node_turtle_control,
		action_node_partol_client,
	])
```

3. 在 CMakeLists.txt中添加代码，将`.launch.py`拷贝到 `install`目录下

```cmake
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)
```

4. 重新 `colcon build`即可

## 使用launch.py设置节点参数

1. 创建参数声明 `action`， 用于解析命令后的参数
   - `generate_launch_description()` 函数中添加以下代码

```python
action_declare_arg_max_speed = launch.actions.DeclareLaunchArgument('launch_max_speed'， default_value='0.0')
```

> `default_value`定义了在命令行中没有提供该参数时的默认值

2. 使用 launch 中参数`launch_max_speed`值来替换节点中的 `max_speed` 参数值

```python
parameters = [{'max_speed'： launch.substitutions.LaunchConfiguration('launch_max_speed'， default='0.0')}]
```

> `default` 定义了获取参数值时的默认值。
>
> 也就是说没有给定 `default`的值，就会使用 `default_value`的值

## 使用动作打印

```python
import ament_index_python import get_package_share_directory

multisim_launch_path = [get_package_share_directory('multisim'), '/launch/', 'multisim.launch.py']
action_log_info = launch.actions.LogInfo(msg=str(multisim_launch_path))

return launch.LaunchDescription([
	action_log_info,
])
```

## 使用动作执行命令

```python
action_ros2_topic_list = launch.actions.ExecuteProcess(
	cmd=['ros2','topic', 'list'],
	output='screen',
)
```

## 使用动作来启动其他launch文件

```python
multisim_launch_path = get_package_share_directory('turtlesim', '/launch', 'multisim.launch.py')
action_include_launch = launch.actions.IncludeLaunchDescription(
	multisim_launch_path
)
```

## 组织动作成一个组，并使用定时器，按制定顺序执行动作

```python
action_group = launch.actions.GroupAction([
	launch.actions.TimerAction(period=2.0, actions=[action_include_launch]),
	launch.actions.TimerAction(period=4.0, actions=[action_ros2_topic_list]),
])

launch.LaunchDescription([
	action_group,
])
```

## 利用条件决定启动那些参数

```python
# 首先声明一个参数动作
action_declare_startup_rqt = launch.actions.DeclareLaunchArgment('startup_rqt', default_value='False')
# 然后通过判断该参数的值来决定是否执行动作
# if startup_rqt 为 True，则执行命令
action_startup_rqt = launch.actions.ExecuteProcess(
	condition=launch.conditions.IfCondition(startup_rqt),
	cmd=['rqt'],
)
```
