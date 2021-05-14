# Azure Cognitive Services LUIS ROS Node
This ROS node bridges between ROS and the Azure Language Understanding Service. The ROS node can be configured to process audio directly from a microphone, or can subscribe to a ROS audio topic, then processes speech and generates "intent" ROS messages which can be processed by another ROS node to generate ROS commands.

> NOTE: This ROS node is in preview.

## CI Status
![Windows CI](https://github.com/ms-iot/ros_msft_luis/workflows/CI/badge.svg)

## Working with LUIS on ROS1
This ROS node works on ROS1 Melodic forward, with Linux and Windows.

>During the preview, this ROS node will be provided as sourcecode. 

### Building on Windows
The Makefile for the LUIS ROS Node will download the LUIS SDK using Nuget.exe when it is present in the path.

Please install ROS1 per the instructions at [Microsoft's ROS Landing page](http://aka.ms/ros)

``` batch
mkdir c:\ws\luis_ws\src
cd c:\ws\luis_ws\src
git clone --recursive https://github.com/ms-iot/ros_msft_luis
git clone https://github.com/ms-iot/audio_common

cd c:\ws\luis_ws
catkin_make
devel\setup.bat
```

### Building on Ubuntu


``` batch
sudo apt-get update
sudo apt-get install build-essential libssl1.0.0 libasound2 wget

mkdir ~/luis_ws/src
cd ~/luis_ws/src
git clone --recursive https://github.com/ms-iot/ros_msft_luis

cd ~/luis_ws
catkin_make
source devel/setup.bash
```
### Running the LUIS ROS Node
The Microsoft Azure Lanugage Understanding Service is an Azure cloud service, which provides Lanugage understanding using sophisticated AI. 

> NOTE: LUIS is available as a containerized deployment; instructions coming soon.

To get started, navigate to the [LUIS console](https://www.luis.ai/), to create a lanugage model. To create your own keyword navigate to the [speech studio](https://speech.microsoft.com/) and create your keyword, download the .table file and place it in a known location. 

Once you have completed the model, you can train and publish to a production slot. This will require you to associate the model with a prediction resource on Azure. Once that has been completed, you can configure the ROS node. The ROS node requires the following information from the [LUIS console](https://www.luis.ai/)

 APP ID -  In Manage Tab --> Settings --> App ID
  
 Primary Key - In the Manage Tab --> Azure Resources --> Prediction Resources --> Primary Key
  
 Location - In the Manage Tab --> Azure Resources --> Rediction Resources --> Location

The ROS node requires the following information from the  portal of [speech studio](https://speech.microsoft.com/portal?noredirect=true)


 Speech Resource Key
  
 Region information example "westus" if it is "West US", "westus2" if it is "West US 2"



The LUIS ROS node can be configured two ways - by embedded in the Azure resource keys in the launch file, or setting them in the environment.

> if you are committing launch files to an open source repo, it is best to use the environment method as to not leak your keys.


Windows:
``` batch
set azure_cs_luis_appid=<Enter your APP ID mentioned above>
set azure_cs_luis_key=<Enter your Primary Key mentioned above>
set azure_cs_luis_region=<Enter your location mentioned above>
set azure_cs_kw_key=<Enter your key mentioned above from the speech studio>
set azure_cs_kw_region=<Enter your location mentioned above from the speech studio>
set azure_cs_kw_path=<Enter the location of the of .table file that you downloaded from above>
set azure_cs_kw=<Enter your custom keyword>
```
> NOTE: You can use the command `setx` instead of `set` to save this to the system environment. However, you'll have to recycle your command window. 

Ubuntu:
``` bash
export azure_cs_luis_appid=<Enter your APP ID mentioned above>
export azure_cs_luis_key=<Enter your Primary Key mentioned above>
export azure_cs_luis_region=<Enter your location mentioned above>
export azure_cs_kw_key=<Enter your key mentioned above from the speech studio>
export azure_cs_kw_region=<Enter your location mentioned above from the speech studio>
export azure_cs_kw_path=<Enter the location of the of .table file that you downloaded from above>
export azure_cs_kw=<Enter your custom keyword>
```
> NOTE: You may wish to place this in your .shellrc file so it is available in each terminal. 

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

### Using a LUIS Container with ROS
The Azure Lanugage Understanding Service (LUIS) supports edge deployments using a container. This model can be deployed directly to the robot if there is sufficient resources, or can be deployed to an edge server or kubernetes cluster.

The language model you have defined is exported from the Azure LUIS portal and injected into the container when started. This container exposes an endpoint in the form of a web url like `http://localhost:5000`. In order for the ROS node to use this endpoint, it must be configured via an environment variable (which is shared by all ROS LUIS instances) or by launch file (which could allow multiple concurrent language models).

Please refer to the [documentation for using a LUIS Container](https://docs.microsoft.com/en-us/azure/cognitive-services/LUIS/luis-container-howto?tabs=v3).

If you chose to use an environment variable, it can be configured in your launch shell or in the system by setting the following environment variables:
Windows:
``` batch
set azure_cs_luis_appid=<guid from your model's appid>
set azure_cs_luis_key=<long number from your subscription>
set azure_cs_luis_region=<region it was deployed in>
set azure_cs_luis_endpoint=<region it was deployed in>
```

Ubuntu:
``` bash
export azure_cs_luis_appid=<guid from your model appid>
export azure_cs_luis_key=<long number from your subscription>
export azure_cs_luis_region=<region it was deployed>
export azure_cs_luis_endpoint=<http://...:port>
```

You can also specify the endpoint parameters in xml:

``` xml
<launch>
  <node name="luis_test" pkg="ros_msft_luis" type="ros_msft_luis_node" output="screen">
    <param name="endpoint" value="<endpoint>" />
    <param name="key" value="<luis key>" />
    <param name="region" value="<luis reguin>" />
    <param name="appid" value="<luis appid>" />
  </node>
</launch>
```

Before launching the ROS node, start the LUIS container. In this case, the exported language model has been downloaded into the ROS workspace root `c:\ws\luis_ws`, and mounted into the `/input` and `/output` directory. (The Language model is an input, logs are output).

``` batch
docker run --rm -it -p 5000:5000 ^
--memory 4g ^
--cpus 2 ^
--mount type=bind,src=c:\ws\luis_ws,target=/input ^
--mount type=bind,src=c:\ws\luis_ws,target=/output ^
mcr.microsoft.com/azure-cognitive-services/luis ^
Eula=accept ^
Billing=https://westus.api.cognitive.microsoft.com/ ^
ApiKey=<your API key> ^
Logging:Console:LogLevel:Default=Debug
```

## Working with LUIS on ROS2
*Coming in Fall 2020*


