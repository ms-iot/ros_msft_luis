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
#include <curl/curl.h>
#include <resource_retriever/retriever.h>

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
std::string g_luisEndpoint;
std::string g_speechEndpoint;
std::string g_kwKey;
std::string g_kwRegion;
std::string g_keyWordPath;
std::string g_keyWord;

std::string DeviceGeometry = ""; // Microphone array device geometry
std::string SelectedGeometry = ""; // Microphone array selected device geometry

std::string g_microphoneTopic;
ros::Publisher g_intent_pub;
ros::Subscriber g_microphone_audio_sub;

int sps;
int bps;

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

    JSON_Value *root_value = json_parse_string(luisJson.c_str());
    JSON_Object *root_object = json_value_get_object(root_value);
    if (root_object)
    {
        // is this "topScoringIntent": "Move Arm Backward" on root?

        JSON_Object *topScoringIntent_object = json_object_dotget_object(root_object, "topScoringIntent");
        if (topScoringIntent_object)
        {
            intent.topIntent = json_object_dotget_string(topScoringIntent_object, "intent");
            if (!intent.topIntent.empty())
            {
                intent.score = (float)json_object_get_number(topScoringIntent_object, "score");

                score = intent.score;
            }
            // Process the entities array
            JSON_Array *entities_array = json_object_get_array(root_object, "entities");
            if (entities_array)
            {
                size_t count = json_array_get_count(entities_array);
                for (size_t i = 0; i < count; i++)
                {
                    JSON_Object *entity_object = json_array_get_object(entities_array, i);
                    if (entity_object)
                    {
                        std::string type = json_object_dotget_string(entity_object, "type");
                        if (type == "builtin.dimension")
                        {
                            // Convert builtin.dimension LUIS entity to message dimension
                            JSON_Object *resolution_object = json_object_dotget_object(entity_object, "resolution");
                            if (resolution_object)
                            {
                                float value = 0.0;
                                const char* value_string = json_object_dotget_string(resolution_object, "value");

                                if (value_string)
                                    value = atof(value_string);

                                std::string unit = json_object_dotget_string(resolution_object, "unit");

                                intent.dimension.value = value;
                                intent.dimension.unit = unit;
                            }
                        }
                        else if (type == "builtin.number")
                        {
                            // Copy all builtin.number LUIS entities to message entities
                            JSON_Object *resolution_object = json_object_dotget_object(entity_object, "resolution");
                            if (resolution_object)
                            {
                                std::string value_string = json_object_dotget_string(resolution_object, "value");
                                std::string subtype = json_object_dotget_string(resolution_object, "subtype");

                                ros_msft_luis_msgs::Entity new_entity;
                                new_entity.name = subtype;
                                new_entity.value = value_string;
                                intent.entities.push_back(new_entity);
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
    g_pushStream->Write(const_cast<uint8_t *>(&msg->data[0]), msg->data.size());
}

// Call LUIS API to retrieve intents from a string of text.
std::string get_intents(std::string text)
{
    resource_retriever::Retriever r;
    resource_retriever::MemoryResource resource;
    CURL *curl = curl_easy_init();

    char* encoded_text = curl_easy_escape(curl, text.c_str(), 0);

    std::string url = g_luisEndpoint + "/luis/v2.0/apps/" + g_luisAppId + "?q=" + encoded_text;

    curl_free(encoded_text);

    try {
        resource = r.get(url); 
    } catch (resource_retriever::Exception& e) {
        ROS_ERROR("Failed to retrieve file: %s", e.what());
        return {};
    }

    std::string result((char *)resource.data.get(), resource.size);

    return result;
}

// Get audio config.
std::shared_ptr<AudioConfig> get_audio_config()
{
    std::shared_ptr<AudioConfig> audioConfig;

    if (!g_microphoneTopic.empty())
    {
        auto streamFormat = AudioStreamFormat::GetWaveFormatPCM(sps, bps, 1);
        g_pushStream = PushAudioInputStream::Create(streamFormat);
        audioConfig = AudioConfig::FromStreamInput(g_pushStream);
    }
    else
    {
        audioConfig = AudioConfig::FromDefaultMicrophoneInput();
    }

    return audioConfig;
}

// Intent recognition using local containers.
void intentRecognitionOffline()
{
    std::shared_ptr<SpeechConfig> config;
    std::promise<void> recognitionEnd;

    config = SpeechConfig::FromEndpoint(g_speechEndpoint);

    // If DeviceGeometry and SelectedGeometry are not empty, we are using the local microphone which is ReSpeaker Mic Array v2.0 (microphone arrays compliant)
    if (!DeviceGeometry.empty() && !SelectedGeometry.empty()){
        config.get()->SetProperty("DeviceGeometry", DeviceGeometry);
        config.get()->SetProperty("SelectedGeometry", SelectedGeometry);
    }

    auto audioConfig = get_audio_config();
    auto recognizer = SpeechRecognizer::FromConfig(config, audioConfig);

    recognizer->Recognizing.Connect([](const SpeechRecognitionEventArgs& e) {
        ROS_INFO("Recognizing: %s", e.Result->Text.c_str());
    });

    recognizer->Recognized.Connect([](const SpeechRecognitionEventArgs& e) {
        if (e.Result->Reason == ResultReason::RecognizedSpeech) {
            ROS_INFO("RECOGNIZED: Text=%s", e.Result->Text.c_str());
            std::string intents_json = get_intents(e.Result->Text);
            if (!intents_json.empty()) {
                ROS_INFO("INTENT: %s", intents_json.c_str());
                parseAndPublishFromJson(intents_json);
            }
        }
        else if (e.Result->Reason == ResultReason::NoMatch) {
            ROS_INFO("NOMATCH: Speech could not be recognized.");
        }
    });

    recognizer->Canceled.Connect([&recognitionEnd](const SpeechRecognitionCanceledEventArgs& e) {
        ROS_INFO("CANCELED: Reason=%d", (int)e.Reason);
        if (e.Reason == CancellationReason::Error) {
            ROS_INFO("CANCELED: ErrorCode=%d", (int)e.ErrorCode);
            ROS_INFO("CANCELED: ErrorDetails=%s", e.ErrorDetails.c_str());
            ROS_INFO("CANCELED: Did you update the speech key and location/region info?");

            recognitionEnd.set_value(); // Notify to stop recognition.
        }
    });

    recognizer->SessionStopped.Connect([&recognitionEnd](const SessionEventArgs& e) {
        ROS_INFO("Session stopped.");
        recognitionEnd.set_value(); // Notify to stop recognition.
    });

    ROS_INFO("Speak into your microphone.");
    auto result = recognizer->RecognizeOnceAsync();

    recognitionEnd.get_future().get();
}

// Keyword-triggered speech recognition using microphone.
void intentRecognition()
{
    std::shared_ptr<SpeechConfig> config;
    std::promise<void> recognitionEnd;

    config = SpeechConfig::FromSubscription(g_luisKey, g_luisRegion);

    auto audioConfig = get_audio_config();
    auto recognizer = IntentRecognizer::FromConfig(config, audioConfig);

    // Creates a Language Understanding model using the app id, and adds specific intents from your model
    auto model = LanguageUnderstandingModel::FromAppId(g_luisAppId);
    recognizer->AddAllIntents(model);

    recognizer->Recognizing.Connect([](const IntentRecognitionEventArgs &e) {
        ROS_INFO("Recognizing: %s", e.Result->Text.c_str());
    });

    recognizer->Recognized.Connect([&](const IntentRecognitionEventArgs &e) {
        if (e.Result->Reason == ResultReason::RecognizedIntent)
        {
            ROS_INFO("RECOGNIZED INTENT: Text = %s", e.Result->Text.c_str());
            ROS_INFO("Intent Id: %s", e.Result->IntentId.c_str());

            std::string luisJson = e.Result->Properties.GetProperty(PropertyId::LanguageUnderstandingServiceResponse_JsonResult);

            ROS_INFO("JSON: %s", luisJson.c_str());
            parseAndPublishFromJson(luisJson);
        }
        else if (e.Result->Reason == ResultReason::RecognizedSpeech)
        {
            ROS_INFO("RECOGNIZED: Text = %s", e.Result->Text.c_str());
        }
        else if (e.Result->Reason == ResultReason::NoMatch)
        {
            ROS_DEBUG("NOMATCH: Speech could not be recognized.");
        }
    });

    recognizer->Canceled.Connect([&recognitionEnd](const IntentRecognitionCanceledEventArgs &e) {
        ROS_DEBUG("CANCELED: Reason=%d", (int)e.Reason);

        if (e.Reason == CancellationReason::Error)
        {
            ROS_DEBUG("CANCELED: ErrorCode=%d", (int)e.ErrorCode);
            ROS_DEBUG("CANCELED: ErrorDetails=%s", e.ErrorDetails.c_str());
            ROS_DEBUG("CANCELED: Did you update the subscription info?");
        }

        recognitionEnd.set_value(); // Notify to stop recognition.
    });

    recognizer->SessionStopped.Connect([&recognitionEnd](const SessionEventArgs &e) {
        ROS_DEBUG("Session stopped.");
        recognitionEnd.set_value(); // Notify to stop recognition.
    });

    recognizer->RecognizeOnceAsync();

    recognitionEnd.get_future().get();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_msft_luis");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    const char *env = std::getenv("azure_cs_luis_key");
    if (env != nullptr)
    {
        g_luisKey = env;
    }

    env = std::getenv("azure_cs_luis_endpoint");
    if (env != nullptr)
    {
        g_luisEndpoint = env;
    }

    env = std::getenv("azure_cs_speech_endpoint");
    if (env != nullptr)
    {
        g_speechEndpoint = env;
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

    env = std::getenv("azure_cs_kw_key");
    if (env != nullptr)
    {
        g_kwKey = env;
    }

    env = std::getenv("azure_cs_kw_region");
    if (env != nullptr)
    {
        g_kwRegion = env;
    }

    env = std::getenv("azure_cs_kw_path");
    if (env != nullptr)
    {
        g_keyWordPath = env;
    }

    env = std::getenv("azure_cs_kw");
    if (env != nullptr)
    {
        g_keyWord = env;
    }
    // Parameters.
    if (g_luisKey.empty() &&
        !nhPrivate.getParam("luiskey", g_luisKey))
    {
        ROS_ERROR("luis key has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_luisRegion.empty() &&
        !nhPrivate.getParam("region", g_luisRegion))
    {
        ROS_ERROR("luis region has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_luisAppId.empty() &&
        !nhPrivate.getParam("appid", g_luisAppId))
    {
        ROS_ERROR("luis AppId has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_kwKey.empty() &&
        !nhPrivate.getParam("kwkey", g_kwKey))
    {
        ROS_ERROR("luis KeyWord key has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_kwRegion.empty() &&
        !nhPrivate.getParam("kwregion", g_kwRegion))
    {
        ROS_ERROR("luis KeyWord region has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_keyWord.empty() &&
        !nhPrivate.getParam("keyword", g_keyWord))
    {
        ROS_ERROR("luis KeyWord value has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_keyWordPath.empty() &&
        !nhPrivate.getParam("keywordpath", g_keyWordPath))
    {
        ROS_ERROR("luis KeyWordPath has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_luisEndpoint.empty())
    {
        nhPrivate.getParam("luisendpoint", g_luisEndpoint);
    }

    if (g_speechEndpoint.empty())
    {
        nhPrivate.getParam("speechendpoint", g_speechEndpoint);
    }

    if ((g_luisEndpoint.empty() && !g_speechEndpoint.empty()) || (!g_luisEndpoint.empty() && g_speechEndpoint.empty())) {
        ROS_ERROR("To use containers, both luisendpoint and speechendpoint must be set");
        nh.shutdown();
        return 0;
    }

    double scoreParam;
    if (nh.getParam("min_score", scoreParam))
    {
        g_minScore = scoreParam;
    }

    if (nh.getParam("mic_topic", g_microphoneTopic))
    {

        if (!nhPrivate.getParam("bps", bps))
        {
            bps = 16;
        }

        if (!nhPrivate.param("sps", sps))
        {
            sps = 16000;
        }

        g_microphone_audio_sub = nh.subscribe(g_microphoneTopic, 1, onAudio);
    }

    g_intent_pub = nh.advertise<ros_msft_luis_msgs::TopIntent>("intent", 1);

    auto config = SpeechConfig::FromSubscription(g_kwKey, g_kwRegion);
    auto kwmodel = KeywordRecognitionModel::FromFile(g_keyWordPath);

    while (ros::ok())
    {
        std::promise<void> recognitionEnd;

        // Creates a speech recognizer using microphone as audio input. The default language is "en-us".
        auto recognizer = SpeechRecognizer::FromConfig(config);

        // Subscribes to events.
        recognizer->Recognizing.Connect([](const SpeechRecognitionEventArgs &e) {
            if (e.Result->Reason == ResultReason::RecognizingKeyword)
            {
                ROS_INFO("RECOGNIZING KEYWORD: Text= %s", e.Result->Text.c_str());
            }
        });

        recognizer->Recognized.Connect([](const SpeechRecognitionEventArgs &e) {
            if (e.Result->Reason == ResultReason::RecognizedKeyword)
            {
                ROS_INFO("RECOGNIZED KEYWORD: Text= %s", e.Result->Text.c_str());
                if (g_luisEndpoint.empty()) {
                    intentRecognition();
                } else {
                    intentRecognitionOffline();
                }
            }
        });

        recognizer->Canceled.Connect([&recognitionEnd](const SpeechRecognitionCanceledEventArgs &e) {
            ROS_DEBUG("CANCELED: Reason=%d", (int)e.Reason);

            if (e.Reason == CancellationReason::Error)
            {
                ROS_DEBUG("CANCELED KEYWORD: ErrorCode=%d", (int)e.ErrorCode);
                ROS_DEBUG("CANCELED KEYWORD: ErrorDetails=%s", e.ErrorDetails.c_str());
                ROS_DEBUG("CANCELED KEYWORD: Did you update the subscription info for the keyword?");
            }
        });

        recognizer->SessionStarted.Connect([&recognitionEnd](const SessionEventArgs &e) {
            ROS_DEBUG("SESSIONSTARTED KEYWORD: SessionId= %s", e.SessionId.c_str());
        });

        recognizer->SessionStopped.Connect([&recognitionEnd](const SessionEventArgs &e) {
            ROS_DEBUG("SESSIONSTOPPED KEYWORD: SessionId= %s", e.SessionId.c_str());

            recognitionEnd.set_value(); // Notify to stop recognition.
        });

        recognizer->StartKeywordRecognitionAsync(kwmodel).wait();

        ROS_INFO("Say something starting with '%s' followed by whatever you want..." , g_keyWord.c_str());

        // Waits for a single successful keyword-triggered speech recognition (or error).
        recognitionEnd.get_future().wait();

        recognizer->StopKeywordRecognitionAsync().wait();
    }

    nh.shutdown();
    return 0;
}
