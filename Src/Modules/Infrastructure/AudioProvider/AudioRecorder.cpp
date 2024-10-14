/**
 * @file AudioRecorder.cpp
 * This file implements a module that provides audio samples using PortAudio.
 */

#include "AudioRecorder.h"
#include "Tools/Settings.h"
#include <iostream>

using std::cout, std::endl;

MAKE_MODULE(AudioRecorder, infrastructure);

std::mutex AudioRecorder::mutex;

AudioRecorder::AudioRecorder() {
    // PortAudio API is not thread-safe.
    std::lock_guard<std::mutex> l(mutex);

    // Initialize PortAudio
    PaError err;
    err = Pa_Initialize();
    AudioRecorder::checkErr(err);

    // Checking number of devices
    int numDevices = Pa_GetDeviceCount();
    OUTPUT_TEXT("Number of devices: %d\n" << numDevices);
    // Print list of audio devices
    for (int i = 0; i < numDevices; i++) {
        const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(i);
        cout << "Device " << i << ": " << deviceInfo->name << endl;
    }
    if (numDevices < 0) {
        printf("Error getting device count.\n");
        exit(EXIT_FAILURE);
    } else if (numDevices == 0) {
        printf("There are no available audio devices on this machine.\n");
        exit(EXIT_SUCCESS);
    }

    const PaDeviceInfo* deviceInfo;

    int device = Pa_GetDefaultInputDevice();
    inputParameters.device = device;
    deviceInfo = Pa_GetDeviceInfo(inputParameters.device);
    OUTPUT_TEXT("Using input device: " << deviceInfo->name);
    inputParameters.channelCount = std::min(static_cast<int>(channels), deviceInfo->maxInputChannels);
    inputParameters.sampleFormat = paFloat32;
    inputParameters.hostApiSpecificStreamInfo = nullptr;
    inputParameters.suggestedLatency = latency;
    err = Pa_OpenStream(
        &stream,
        &inputParameters,
        nullptr,
        sampleRate,
        paFramesPerBufferUnspecified, //TODO: change to FRAMES_PER_BUFFER?
        paNoFlag,
        nullptr,
        nullptr
    );
    OUTPUT_TEXT("Stream open: " << (stream ? "true" : "false"));
    checkErr(err);
}

AudioRecorder::~AudioRecorder() {
    std::lock_guard<std::mutex> l(mutex);

    if (stream) {
        if (Pa_IsStreamActive(stream))
        Pa_StopStream(stream);

        Pa_CloseStream(stream);
        stream = nullptr;
    }

    Pa_Terminate();
}

void AudioRecorder::checkErr(PaError err) {
    if (err != paNoError) {
        OUTPUT_TEXT("PortAudio error: %s\n" << Pa_GetErrorText(err));
        exit(EXIT_FAILURE);
    }
}

void AudioRecorder::update(AudioData& audioData) {
    audioData.isValid = false;
    audioData.samples.clear();

    if (!stream) return;

    PaError err;

    // Start stream and fill audiodata just once
    if (Pa_IsStreamStopped(stream)) {
        OUTPUT_TEXT("AudioRecorder: Starting stream");
        const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(inputParameters.device);
        const PaStreamInfo* streamInfo = Pa_GetStreamInfo(stream);
        audioData.device = deviceInfo->name;
        audioData.api = Pa_GetHostApiInfo(deviceInfo->hostApi)->name;
        audioData.latency = streamInfo->inputLatency;
        audioData.channels = inputParameters.channelCount;
        audioData.sampleRate = static_cast<unsigned>(streamInfo->sampleRate);
        err = Pa_StartStream(stream);
        if (err != paNoError) {
            std::lock_guard<std::mutex> l(mutex);
            OUTPUT_ERROR("AudioRecorder: Pa_StartStream failed: " << Pa_GetErrorText(err) << "(" << err << ")");
            Pa_CloseStream(stream);
            stream = nullptr;
            return;
        }
    }

    signed long available = Pa_GetStreamReadAvailable(stream);
    if (available < 0) {
        OUTPUT_ERROR("AudioRecorder: Pa_GetStreamReadAvailable failed: " << Pa_GetErrorText(static_cast<PaError>(available)) << "(" << static_cast<PaError>(available) << ")");
        return;
    }

    // restart stream if no data is available over a longer period of time
    if (available == 0)
    {
        if (++noDataCount > 30)
        {
        OUTPUT_WARNING("AudioRecorder: Data timeout, restarting stream");
        Pa_StopStream(stream);
        noDataCount = 0;
        }
        return;
    }
    noDataCount = 0;

    audioData.samples.resize(available * audioData.channels);
    err = Pa_ReadStream(stream, audioData.samples.data(), available);
    if (err != paNoError) {
        OUTPUT_TEXT("AudioRecorder: Pa_ReadStream failed: " << Pa_GetErrorText(err) << "(" << err << ")");
        audioData.samples.clear();
        // restart stream immediately after timeout
        if (err == paTimedOut)
        Pa_StopStream(stream);
        return;
    }
    audioData.isValid = true;
}
