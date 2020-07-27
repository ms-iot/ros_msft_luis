//
// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE.md file in the project root for full license information.
//
#include <ros/ros.h>
#include <windows.h>

#include <iostream>
#include <speechapi_cxx.h>
#include <parson.h>

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;
using namespace Microsoft::CognitiveServices::Speech::Audio;
using namespace Microsoft::CognitiveServices::Speech::Intent;

std::string g_luisKey;
std::string g_luisRegion;
std::string g_luisAppId;
std::string g_luisIntent;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "azure_cs_luis");
    ros::NodeHandle nh;
    std::promise<void> recognitionEnd;

    g_luisKey = std::getenv("azure_cs_luis_key");
    g_luisAppId = std::getenv("azure_cs_luis_appid");
    g_luisRegion = std::getenv("azure_cs_luis_region");
    g_luisIntent = std::getenv("azure_cs_luis_intent");

    // Parameters.
    if (g_luisKey.empty() ||
        nh.getParam("key", g_luisKey))
    {
        ROS_ERROR("luis key has not been set");
        nh.shutdown();
        return 0;
    }
    
    if (g_luisRegion.empty() ||
        nh.getParam("region", g_luisRegion))
    {
        ROS_ERROR("luis region has not been set");
        nh.shutdown();
        return 0;
    }

    if (g_luisIntent.empty() ||
        nh.getParam("intent", g_luisIntent))
    {
        ROS_ERROR("luis intent has not been set");
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

    // Creates an instance of a speech config with specified subscription key and service region.
    // Replace with your own subscription key and service region (e.g., "westus").
    auto config = SpeechConfig::FromAuthorizationToken(g_luisKey.c_str(), g_luisRegion.c_str());

    // Creates an intent recognizer using microphone as audio input.
    auto recognizer = IntentRecognizer::FromConfig(config);

    // Creates a Language Understanding model using the app id, and adds specific intents from your model
    auto model = LanguageUnderstandingModel::FromAppId(g_luisAppId);
    recognizer->AddIntent(model, g_luisIntent, "intent");

    while (ros::ok())
    {
        // Subscribes to events.
        recognizer->Recognizing.Connect([] (const IntentRecognitionEventArgs& e)
        {
            //cout << "Recognizing:" << e.Result->Text << std::endl;
        });

        recognizer->Recognized.Connect([] (const IntentRecognitionEventArgs& e)
        {
            if (e.Result->Reason == ResultReason::RecognizedIntent)
            {
                // cout << "RECOGNIZED: Text=" << e.Result->Text << std::endl;
                // cout << "  Intent Id: " << e.Result->IntentId << std::endl;
                // cout << "  Intent Service JSON: " << e.Result->Properties.GetProperty(PropertyId::LanguageUnderstandingServiceResponse_JsonResult) << std::endl;
            }
            else if (e.Result->Reason == ResultReason::RecognizedSpeech)
            {
                // cout << "RECOGNIZED: Text=" << e.Result->Text << " (intent could not be recognized)" << std::endl;
            }
            else if (e.Result->Reason == ResultReason::NoMatch)
            {
                // cout << "NOMATCH: Speech could not be recognized." << std::endl;
            }
        });

        recognizer->Canceled.Connect([&recognitionEnd](const IntentRecognitionCanceledEventArgs& e)
        {
            // cout << "CANCELED: Reason=" << (int)e.Reason << std::endl;

            if (e.Reason == CancellationReason::Error)
            {
                // cout << "CANCELED: ErrorCode=" << (int)e.ErrorCode << std::endl;
                // cout << "CANCELED: ErrorDetails=" << e.ErrorDetails << std::endl;
                // cout << "CANCELED: Did you update the subscription info?" << std::endl;
            }

            recognitionEnd.set_value(); // Notify to stop recognition.
        });

        recognizer->SessionStopped.Connect([&recognitionEnd](const SessionEventArgs& e)
        {
            // cout << "Session stopped.";
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
