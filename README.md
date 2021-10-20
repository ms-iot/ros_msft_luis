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

To get started with LUIS and the Speech Services, check [this guideline](./docs/walkthrough-luis-config.md).

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
set azure_cs_stop_kw_path=<Enter the location of the of .table file of the Stop Keyword that you downloaded from above>
set azure_cs_kw=<Enter your custom keyword>
set azure_cs_stop_kw=<Enter your stop custom keyword>
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
export azure_cs_stop_kw_path=<Enter the location of the of .table file of the Stop Keyword that you downloaded from above>
export azure_cs_kw=<Enter your custom keyword>
export azure_cs_stop_kw=<Enter your stop custom keyword>
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

# Using the `move_base` node

The `ros_msft_luis_move_base` node is provided to translate the intents into `move_base` navigation goals. It currently understands the following intents:

- "Move Forward" / "Move Backward", followed by a distance expressed in meters, feet or yards. For example: "move forward 2 meters", "move backward 3 feet".
- "Turn Left" / "Turn Right", followed by an angle expressed in degrees. For example: "turn left 34 degrees". If no angle is given, a default of 90 degrees will be used, so you can just say "turn right" for example.
- "Stop", which cancels all current navigation.

To launch the node, you can use the provided launch configuration:

```
roslaunch ros_msft_luis luis_move_base.launch
```

### Using containers with ROS

The Azure Cognitive Services support edge deployments using [containers](https://docs.microsoft.com/en-us/azure/cognitive-services/cognitive-services-container-support), including Speech-to-Text and Language Understanding Service (LUIS). These models can be deployed directly to the robot if there are sufficient resources, or can be deployed to an edge server or Kubernetes cluster.

**Starting the Speech-to-Text container**

You will first need to run the Speech container. Please refer to the [documentation for using Speech Services containers](https://docs.microsoft.com/en-us/azure/cognitive-services/speech-service/speech-container-howto?tabs=stt%2Ccsharp%2Csimple-format).

You will need to select a local port to expose the Speech service; in this example, 5000.

``` shell
docker run --rm -it -p 5000:5000 \
--memory 4g --cpus 4 \
mcr.microsoft.com/azure-cognitive-services/speechservices/speech-to-text \
Eula=accept \
Billing=<your endpoint URI> \
ApiKey=<your API key>

```

**Starting the LUIS container**

The language model you have defined is exported from the Azure LUIS portal and injected into the container when started. Please refer to the [documentation for using a LUIS Container](https://docs.microsoft.com/en-us/azure/cognitive-services/LUIS/luis-container-howto?tabs=v3).

You will find below an example running the LUIS node using Docker. In this case, the exported language model has been downloaded into the ROS workspace root `c:\ws\luis_ws`, and mounted into the `/input` and `/output` directory. (the language model is an input, logs are output).

Make sure to use a different local port than the one used for the Speech container: in this example, 5001.

``` batch
docker run --rm -it -p 5001:5000 ^
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

**Starting the ROS node**

The containers expose endpoints in the form of URLs like these:

- LUIS: `http://localhost:5001`
- Speech: `ws://localhost:5000/speech/recognition/conversation/cognitiveservices/v1`

In order for the ROS node to use these endpoints, they must be configured via additional environment variables (which are shared by all ROS LUIS instances) or by ladditional parameters in the aunch file (which could allow multiple concurrent language models).

Make sure to use the right local ports, defined when running the containers!

If you chose to use environment variables, they can be configured in your launch shell or in the system by setting the following environment variables:

Windows:

``` batch
set azure_cs_luis_endpoint=http://host:port
set azure_cs_speech_endpoint=ws://host:port/speech/recognition/conversation/cognitiveservices/v1
```

Ubuntu:

``` bash
export azure_cs_luis_endpoint=http://host:port
export azure_cs_speech_endpoint=ws://host:port/speech/recognition/conversation/cognitiveservices/v1
```

You can also specify the endpoint parameters in the XML launch file:

``` xml
<launch>
  <node name="luis_test" pkg="ros_msft_luis" type="ros_msft_luis_node" output="screen">
    <param name="luisendpoint" value="<endpoint>" />
    <param name="speechendpoint" value="<endpoint>" />
    <!-- other configuration parameters as defined above -->
  </node>
</launch>
```

You can then run the ROS node as previously.

```
roslaunch ros_msft_luis luis.launch
```

## Working with LUIS on ROS2

*Coming in Fall 2020*
