//
// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE.md file in the project root for full license information.
//
#include <ros/ros.h>
#ifdef WIN32
#include <windows.h>
#endif

#include <iostream>
#include <speechapi_cxx.h>
#include <parson.h>
#include <audio_common_msgs/AudioData.h>


#include <ros_msft_luis_msgs/Entity.h>
#include <ros_msft_luis_msgs/Intent.h>
#include <ros_msft_luis_msgs/TopIntent.h>

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;
using namespace Microsoft::CognitiveServices::Speech::Intent;

std::string g_luisKey;
std::string g_luisRegion;
std::string g_luisAppId;

std::string g_microphoneTopic;

ros::Publisher g_intent_pub;
ros::Subscriber g_microphone_audio_sub;

std::shared_ptr<PushAudioInputStream> g_pushStream;

double g_minScore = 0.8;

/* Example response
{
  "query": "move forward 1 foot",
  "topScoringIntent": {
    "intent": "Move forward",
    "score": 0.854586
  },
  "entities": [
    {
      "entity": "1 foot",
      "type": "builtin.dimension",
      "startIndex": 13,
      "endIndex": 18,
      "resolution": {
        "unit": "Foot",
        "value": "1"
      }
    },
    {
      "entity": "foot",
      "type": "builtin.keyPhrase",
      "startIndex": 15,
      "endIndex": 18
    },
    {
      "entity": "1",
      "type": "builtin.number",
      "startIndex": 13,
      "endIndex": 13,
      "resolution": {
        "subtype": "integer",
        "value": "1"
      }
    }
  ]
}
*/


void parseAndPublishFromJson(std::string luisJson)
{
    double score = 0.0;
    ros_msft_luis_msgs::TopIntent intent;

    JSON_Value* root_value = json_parse_string(luisJson.c_str());
    JSON_Object* root_object = json_value_get_object(root_value);
    if (root_object)
    {
        // is this "topScoringIntent": "Move Arm Backward" on root?

        JSON_Object* topScoringIntent_object = json_object_dotget_object(root_object, "topScoringIntent");
        if (topScoringIntent_object)
        {
            intent.topIntent = json_object_dotget_string(topScoringIntent_object, "intent");
            if (!intent.topIntent.empty())
            {
                intent.score = (float)json_object_get_number(topScoringIntent_object, "score"); 

                score = intent.score;
            }
            // Process the entities array
            JSON_Array* entities_array = json_object_get_array(root_object, "entities");
            if (entities_array)
            {
                size_t count = json_array_get_count(entities_array);
                for (size_t i = 0; i < count; i++)
                {
                    JSON_Object* entity_object = json_array_get_object(entities_array, i);
                    if (entity_object)
                    {
                        std::string type = json_object_dotget_string(entity_object, "type");
                        if (type == "builtin.dimension")
                        {
                            JSON_Object* resolution_object = json_object_dotget_object(entity_object, "resolution");
                            if (resolution_object)
                            {
                                float value = atof(json_object_dotget_string(resolution_object, "value")); 
                                std::string unit = json_object_dotget_string(resolution_object, "unit");

                                intent.dimension.value = value;
                                intent.dimension.unit = unit;
                            }
                        }
                    }
                }
            }

            intent.score = (float)json_object_get_number(topScoringIntent_object, "score"); 
        }
    }

    if (!intent.topIntent.empty() &&
        score > g_minScore)
    {
        g_intent_pub.publish(intent);
    }
}

void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
{
    // push stream signature is not const correct, but does not modify the buffer.
    g_pushStream->Write(const_cast<uint8_t*>(&msg->data[0]), msg->data.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_msft_luis");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    std::promise<void> recognitionEnd;

    const char* env = std::getenv("azure_cs_luis_key");
    if (env != nullptr)
    {
        g_luisKey = env;
    }

    env = std::getenv("azure_cs_luis_appid");
    if (env != nullptr)
    {
        g_luisAppId = env;
    }

    env = std::getenv("azure_cs_luis_region");
    if (env != nullptr)
    {
        g_luisRegion = env;
    }
    
    // Parameters.
    if (g_luisKey.empty() ||
        nhPrivate.getParam("key", g_luisKey))
    {
        ROS_ERROR("luis key has not been set");
        nh.shutdown();
        return 0;
    }
    
    if (g_luisRegion.empty() ||
        nhPrivate.getParam("region", g_luisRegion))
    {
        ROS_ERROR("luis region has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_luisAppId.empty() ||
        nh.getParam("AppId", g_luisAppId))
    {
        ROS_ERROR("luis AppId has not been set");
        nh.shutdown();
        return 0;
    }

    double scoreParam;
    if (nh.getParam("min_score", scoreParam))
    {
        g_minScore = scoreParam;
    }

    g_intent_pub = nh.advertise<ros_msft_luis_msgs::TopIntent>("intent", 1);


    // Creates an instance of a speech config with specified subscription key and service region.
    // Replace with your own subscription key and service region (e.g., "westus").
    auto config = SpeechConfig::FromSubscription(g_luisKey.c_str(), g_luisRegion.c_str());

    std::shared_ptr<IntentRecognizer> recognizer;

    if (nh.getParam("mic_topic", g_microphoneTopic))
    {
        int sps;
        int bps;
        
        if (!nhPrivate.getParam("bps", bps))
        {
            bps = 16;
        }
        
        if (!nhPrivate.param("sps", sps))
        {
            sps = 16000;
        }

        // channels?
        auto streamFormat = AudioStreamFormat::GetWaveFormatPCM(sps, bps, 1);
        g_pushStream = PushAudioInputStream::Create(streamFormat);
        auto audioConfig = AudioConfig::FromStreamInput(g_pushStream);

        recognizer = IntentRecognizer::FromConfig(config, audioConfig);

        g_microphone_audio_sub = nh.subscribe(g_microphoneTopic, 1, onAudio);
    }
    else
    {
        // Creates an intent recognizer using the default microphone as audio input.
        recognizer = IntentRecognizer::FromConfig(config);
    }

    // Creates a Language Understanding model using the app id, and adds specific intents from your model
    auto model = LanguageUnderstandingModel::FromAppId(g_luisAppId);
    recognizer->AddAllIntents(model);

    while (ros::ok())
    {
        // Subscribes to events.
        recognizer->Recognizing.Connect([] (const IntentRecognitionEventArgs& e)
        {
            ROS_INFO("Recognizing: %s", e.Result->Text.c_str());
        });

        recognizer->Recognized.Connect([] (const IntentRecognitionEventArgs& e)
        {
            if (e.Result->Reason == ResultReason::RecognizedIntent)
            {
                ROS_INFO("RECOGNIZED: Text = %s", e.Result->Text.c_str());
                ROS_INFO("Intent Id: %s", e.Result->IntentId.c_str());

                std::string luisJson = e.Result->Properties.GetProperty(PropertyId::LanguageUnderstandingServiceResponse_JsonResult);

                ROS_INFO("JSON: %s", luisJson.c_str());
                parseAndPublishFromJson(luisJson);
            }
            else if (e.Result->Reason == ResultReason::RecognizedSpeech)
            {
                ROS_INFO("RECOGNIZED: Text= %s", e.Result->Text.c_str());
            }
            else if (e.Result->Reason == ResultReason::NoMatch)
            {
                ROS_DEBUG("NOMATCH: Speech could not be recognized.");
            }
        });

        recognizer->Canceled.Connect([&recognitionEnd](const IntentRecognitionCanceledEventArgs& e)
        {
            ROS_DEBUG("CANCELED: Reason=%d", (int)e.Reason);

            if (e.Reason == CancellationReason::Error)
            {
                ROS_DEBUG("CANCELED: ErrorCode=%d", (int)e.ErrorCode);
                ROS_DEBUG("CANCELED: ErrorDetails=%s", e.ErrorDetails.c_str());
                ROS_DEBUG("CANCELED: Did you update the subscription info?");
            }

            recognitionEnd.set_value(); // Notify to stop recognition.
        });

        recognizer->SessionStopped.Connect([&recognitionEnd](const SessionEventArgs& e)
        {
            ROS_DEBUG("Session stopped.");
            recognitionEnd.set_value(); // Notify to stop recognition.
        });    
        
        recognizer->StartContinuousRecognitionAsync();

        recognitionEnd.get_future().get();

        ros::spin();
    }

    recognizer->StopContinuousRecognitionAsync();
    nh.shutdown();
    return 0;
}
