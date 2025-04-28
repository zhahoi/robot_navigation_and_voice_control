# robot_navigation_and_voice_control
在ROS环境下进行机器人建图导航(Cartographer+Movebase)、目标检测（Nanodet)、语音播报和语音导航任务发布(sherpa_onnx)。

本仓库代码用于学习ROS环境下，进行机器人建图、导航、目标检测。此外，还集成了目标检测结果播报和通过检测关键词控制机器人运动的功能。

本仓库中代码为部分功能代码，由于复用了本人其他仓库的代码[cartographer_3d_gazebo](https://github.com/zhahoi/cartographer_3d_gazebo)，因此不包含机器人的建图和导航功能。但理论上只要是任意的包含建图和导航功能的机器人均可（为了方便实现后续的目标检测，机器人至少需要包含一个相机，以便于订阅相机话题）。此外，希望机器人可以通过`/cmd_vel`控制机器人运动。



### 测试环境

- 测试系统：ubuntu 20.04

- ros版本： noetic

  

### 基础配置

将本仓库代码先克隆到与你机器人建图和导航相同工作空间的`src`目录下，为了能够让代码后续可以成功编译，你还需要进行以下操作：

**1.安装语音播报和语音导航依赖**

```sh
$ pip install sherpa_onnx==1.10.35 -i https://mirrors.cloud.tencent.com/pypi/simple 
$ pip install soundfile -i https://mirrors.cloud.tencent.com/pypi/simple 
$ pip install sounddevice -i https://mirrors.cloud.tencent.com/pypi/simple 

$ sudo apt install libportaudio2
```

**2.安装目标检测依赖和修改权重文件路径**

为了能够让目标检测功能能够低成本运行，本仓库选择的目标检测模型为Nanodet。需要提前安装的依赖库为OpenCV和NCNN，然后修改`nanodet`文件夹下的'**CMakeLists.txt**'文件，修改依赖库的绝对路径：

```cmake
set(OpenCV_DIR "/home/hit/Softwares/opencv-3.4.10/build")
find_package(OpenCV REQUIRED)

set(ncnn_DIR "/home/hit/Softwares/ncnn/build/install/lib/cmake/ncnn")
find_package(ncnn REQUIRED)
if(NOT TARGET ncnn)
    message(WARNING "ncnn NOT FOUND! Please set ncnn_DIR environment variable")
else()
    message("ncnn found")
endif()
```

此外为了能够成功加载Nanodet的权重文件，还需修改Nanodet的权重路径。具体操作为：先进入到`nanodet/include/nanodet`路径下,用编辑器打开`nanodet.h`文件，然后修改

```c++
#define PARAM_PATH "/home/hit/cartographer_ws/src/nanodet/weights/nanodet-m.param"
#define BIN_PATH "/home/hit/cartographer_ws/src/nanodet/weights/nanodet-m.bin"
```

为真实的权重路径。

**3.编译**

返回到整个工作空间的根目录下，执行'**catkin_make_isolated**'便可进行编译。（如果使用自己的建图和导航方法，应该是执行'**catkin_make**'进行编译。）



### 建图和导航

由于是复用了本人其他仓库的代码[cartographer_3d_gazebo](https://github.com/zhahoi/cartographer_3d_gazebo)，建图和导航功能可以参考该仓库的README实现。

执行导航的命令为：

```sh
$source devel_isolated/setup.bash 
$roslaunch scout_gazebo scout_gazebo.launch 

$rosrun teleop_twist_keyboard teleop_twist_keyboard.py

$source devel_isolated/setup.bash 
$roslaunch cartographer_ros demo_3d_localization.launch 

$source devel_isolated/setup.bash 
$roslaunch cartographer_ros move_base.launch 
```



### 目标检测

本仓库的目标检测功能，暂设置只针对于**人**进行检测，因为在gazebo仿真环境中很难找到coco数据集中包含的其他检测类别。

`nanodet/config/config.yaml`文件，可以修改图像话题等的超参数：

```yaml
nanodet_ros:
  image_topic: "/camera/rgb/image_raw"        # image topic
  target_size: 320
  prob_threshold: 0.4
  nms_threshold: 0.5
  use_gpu: false
  downsampling_infer: 10      
```

此外可以修改`src/nanodet_node.cpp`设置检测的类别：

```c++
if (object_.label == 0)
{
	ros::Time current_time = ros::Time::now();
	if ((current_time - last_tts_publish_time_).toSec() >= tts_publish_interval_) 
	{
		std_msgs::String msg;
		msg.data = "检测到行人";
		m_labels_pub.publish(msg);
		last_tts_publish_time_ = current_time;  // 更新时间戳
	}
}
```

执行目标检测的命令为：

```shell
$source devel_isolated/setup.bash 
$roslaunch nanodet run.launch 
```

![图像1]([C:\Users\HIT-HAYES\Desktop\图像1.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F1.png))

![图像2]([C:\Users\HIT-HAYES\Desktop\图像2.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F2.png))

![图像4]([C:\Users\HIT-HAYES\Desktop\图像4.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F4.png))



### 语音播报

本仓库的语音播报（tts)功能，参考自[sherpa_onnx_ros](https://gitee.com/bingda-robot/sherpa_onnx_ros.git)，tts订阅的话题为`/tts_input`，可以实时播报发布的文字信息。

执行语音播报的命令为：

```sh
$source devel_isolated/setup.bash
$roslaunch sherpa_onnx_ros tts.launch 
```

当目标检测检测到"**人**"之后，会女声播报“**检测到行人**”的语音。

![图像3]([C:\Users\HIT-HAYES\Desktop\图像3.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F3.png))



### 语音导航任务发布

本仓库的导航任务发布分为两种，一种为给机器人发布“前进、后退、左转、右转”等的简单指令，另一种为在机器人导航的条件下，通过发布任务让机器人到达特定的目标点。

通过从麦克风用普通话说出下列的关键词，可以让机器人执行相应的任务。

![图像5]([C:\Users\HIT-HAYES\Desktop\图像5.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F5.png))

可以从`sherpa-onnx-ros/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01/voice_commands.yaml`下修改机器人的线速度、角速度和导航的目标点。

```yaml
# 线速度x, 角速度z
command_map:
  前进: [0.5, 0.0]   
  停止: [0.0, 0.0]
  左转: [0.2, 0.5]    # 低速前进+左转
  右转: [0.2, -0.5]
  后退: [-0.3, 0.0]   # 负值表示反向
  # 加速: [0.8, 0.0]    # 高速前进

# goal.pose.position.x = self.nav_goals[command][0]
# goal.pose.position.y = self.nav_goals[command][1]
# goal.pose.orientation.z = self.nav_goals[command][2]
# goal.pose.orientation.w = self.nav_goals[command][3]
nav_goals:
  去一号目标点: [7.42, -8.23, -0.99, 0.06]
  去二号目标点: [9.56, -3.34, 0.70, 0.71]
  去三号目标点: [18.57, -4.71, -0.69, 0.71]
  回到原点: [0.0, 0.0, -0.71, 0.70]
  取消导航: [0.0, 0.0, 0.0, 0.0]
```

执行语音导航任务的命令为：

```sh
$source devel_isolated/setup.bash
$roslaunch sherpa_onnx_ros keywords.launch 
```

此外你可以客制化自己的检测命令，可以参考以下博客文章[kaldi sherpa-onnx 生成keyword.txt](https://blog.csdn.net/hongkid/article/details/143106261),只是效果因指令而异，需要具体进行测试。

![图像6]([C:\Users\HIT-HAYES\Desktop\图像6.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F6.png))

![图像7]([C:\Users\HIT-HAYES\Desktop\图像7.png](https://github.com/zhahoi/robot_navigation_and_voice_control/blob/main/images/%E5%9B%BE%E5%83%8F7.png))



### 写在最后

创作不易，如果觉得这个仓库还可以的话，麻烦给一个star，这就是对我最大的鼓励。



### Reference

-[sherpa_onnx_ros](https://gitee.com/bingda-robot/sherpa_onnx_ros.git)

-[cartographer_3d_gazebo](https://github.com/zhahoi/cartographer_3d_gazebo)
