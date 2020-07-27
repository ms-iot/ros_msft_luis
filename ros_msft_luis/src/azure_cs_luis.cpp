//
// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE.md file in the project root for full license information.
//
#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS 1
#define _CRT_SECURE_NO_WARNINGS

#include <ros/ros.h>
#include <windows.h>

#include <iostream>
#include <speechapi_cxx.h>

using namespace std;
using namespace Microsoft::CognitiveServices::Speech;

std::string g_luisKey;
std::string g_luisRegion;

void recognizeSpeech()
{
    // Creates an instance of a speech config with specified subscription key and service region.
    // Replace with your own subscription key and service region (e.g., "westus").
    auto config = SpeechConfig::FromAuthorizationToken(g_luisKey.c_str(), g_luisRegion.c_str());

    // Creates a speech recognizer.
    auto recognizer = SpeechRecognizer::FromConfig(config);
    ROS_INFO("Waiting for utterance\n");

    // Performs recognition. RecognizeOnceAsync() returns when the first utterance has been recognized,
    // so it is suitable only for single shot recognition like command or query. For long-running
    // recognition, use StartContinuousRecognitionAsync() instead.
    auto result = recognizer->RecognizeOnceAsync().get();

    // Checks result.
    if (result->Reason == ResultReason::RecognizedSpeech)
    {
        ROS_INFO("We recognized: %s", result->Text);
    }
    else if (result->Reason == ResultReason::NoMatch)
    {
        ROS_INFO("NOMATCH: Speech could not be recognized.");
    }
    else if (result->Reason == ResultReason::Canceled)
    {
        auto cancellation = CancellationDetails::FromResult(result);
        ROS_INFO("CANCELED: Reason=%d", (int)cancellation->Reason);

        if (cancellation->Reason == CancellationReason::Error) 
        {
            ROS_INFO("CANCELED: ErrorCode= %d", (int)cancellation->ErrorCode);
            ROS_INFO("CANCELED: ErrorDetails= %s", cancellation->ErrorDetails);
            ROS_INFO("CANCELED: Did you update the subscription info?");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "azure_cs_luis");
    ros::NodeHandle nh;

    g_luisKey = std::getenv("azure_cs_luis_key");
    g_luisRegion = std::getenv("azure_cs_luis_region");;

    // Parameters.
    if (g_luisKey.empty() ||
        nh.getParam("azure_cs_luis_key", g_luisKey))
    {
        ROS_ERROR("luis key has not been set");
        nh.shutdown();
        return 0;
    }
    
    if (g_luisRegion.empty() ||
        nh.getParam("azure_cs_luis_region", g_luisRegion))
    {
        ROS_ERROR("luis region has not been set");
        nh.shutdown();
        return 0;
    }

    while (ros::ok())
    {
        recognizeSpeech();
        ros::spinOnce();
    }

    nh.shutdown();
    return 0;
}
