# Azure Cognitive Services LUIS ROS Node
This ROS node bridges between ROS and the Azure Language Understanding Service. The ROS node can be configured to process audio directly from a microphone, or can subscribe to a ROS audio topic, then processes speech and generates "intent" ROS messages which can be processed by another ROS node to generate ROS commands.

> NOTE: This ROS node is in preview.

## Working with LUIS on ROS1
This ROS node works on ROS1 Melodic forward, with Linux and Windows.

>During the preview, this ROS node will be provided as sourcecode. 

### Building on Windows
The Makefile for the LUIS ROS Node will download the LUIS SDK using Nuget.exe when it is present in the path.

Please install ROS1 per the instructions at [Microsoft's ROS Landing page](http://aka.ms/ros)

``` batch
choco install wget
wget https://dist.nuget.org/win-x86-commandline/latest/nuget.exe -O c:\opt\ros\melodic\x64\bin\nuget.exe

mkdir c:\ws\luis_ws\src
cd c:\ws\luis_ws\src
git clone --recusive https://github.com/ms-iot/ros_msft_luis
git clone https://github.com/ms-iot/audio_common

cd c:\ws\luis_ws
catkin_make
devel\setup.bat
```

### Building on Ubuntu

Before building the LUIS node on Linux, please install the LUIS Linux SDK per the [sample instructions](https://github.com/Azure-Samples/cognitive-services-speech-sdk/tree/master/quickstart/cpp/linux/from-microphone)


``` batch
sudo apt-get update
sudo apt-get install build-essential libssl1.0.0 libasound2 wget

mkdir ~/speechsdk
export SPEECHSDK_ROOT="~/speechsdk"
mkdir -p "$SPEECHSDK_ROOT"
wget -O SpeechSDK-Linux.tar.gz https://aka.ms/csspeech/linuxbinary
tar --strip 1 -xzf SpeechSDK-Linux.tar.gz -C "$SPEECHSDK_ROOT"


mkdir ~/luis_ws/src
cd ~/luis_ws/src
git clone --recusive https://github.com/ms-iot/ros_msft_luis

cd ~/luis_ws
catkin_make
source devel/setup.bash
```
### Running the LUIS ROS Node
The Microsoft Azure Lanugage Understanding Service is an Azure cloud service, which provides Lanugage understanding using sophisticated AI. 

> NOTE: LUIS is available as a containerized deployment; instructions coming soon.

To get started, navigate to the [LUIS console](https://www.luis.ai/), to create a lanugage model.

Once you have completed the model, you can train and publish to a production slot. This will require you to associate the model with a prediction resource on Azure. Once that has been completed, you can configure the ROS node.

The LUIS ROS node can be configured two ways - by embedded in the Azure resource keys in the launch file, or setting them in the environment.

> if you are committing launch files to an open source repo, it is best to use the environment method as to not leak your keys.


Windows:
``` batch
set azure_cs_luis_appid=<guid from your model's appid>
set azure_cs_luis_key=<long number from your subscription>
set azure_cs_luis_region=<region it was deployed in>
```

Ubuntu:
``` bash
export azure_cs_luis_appid=guid from your model appid
export azure_cs_luis_key=long number from your subscription
export azure_cs_luis_region=region it was deployed
```

```
roslaunch ros_msft_luis luis.launch
```

If you would like to use a custom microphone, such as the Respeaker which is available on the [Hello Robot](https://hello-robot.com/product), you can set the subscription topic in the launch file:

``` xml
<launch>
  <node name="luis_test" pkg="ros_msft_luis" type="ros_msft_luis_node" output="screen">
    <arg name="mic_topic" value="/audio">
  </node>
</launch>
```


## Working with LUIS on ROS2
*Coming in Fall 2020*


