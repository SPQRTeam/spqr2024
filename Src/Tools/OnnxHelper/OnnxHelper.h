/**
 * @file OnnxHelper.h
 *
 * @author Filippo Ansalone
 */

#pragma once

#include <cassert>
#include <vector>
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <iostream>

template<typename inputType, typename outputType> class OnnxHelper
{
private:
    Ort::Env env;

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::unique_ptr<Ort::Session> session;
    int inputSize;
    int outputSize;
    std::vector<int64_t> inputShape;
    std::vector<int64_t> outputShape;
    std::unique_ptr<char*[]> inputNames;
    std::unique_ptr<char*[]> outputNames;

    void setTensorsNames();
    void setTensorsShapes();
    
public:
    /**
     * Creates the OnnxRuntime session and set static parameters
     * 
     * @param path The path to a valid .onnx file (e.g. "/Config/NeuralNets/....")
     */
    OnnxHelper(std::string path);

    /**
     * Run inference of the loaded model
     * 
     * @param data Pointer to the first element of the input data
     * @param outputBuffer Pointer to the first block of the output buffer (already allocated)
     */
    void infer(void* data, void* outputBuffer);
};

template<typename inputType, typename outputType> void OnnxHelper<inputType, outputType>::setTensorsNames()
{
    Ort::AllocatorWithDefaultOptions allocator;

    size_t numInputNodes = session->GetInputCount();
    inputNames = std::make_unique<char*[]>(numInputNodes);
    for (size_t i = 0; i < numInputNodes; i++) {
        std::string inputName = session->GetInputNameAllocated(i, allocator).get();
        inputNames[i] = strdup(inputName.c_str());
    }

    size_t numOutputNodes = session->GetOutputCount();
    outputNames = std::make_unique<char*[]>(numOutputNodes);
    for (int i = 0; i < numOutputNodes; i++){
        std::string outputName = session->GetOutputNameAllocated(i, allocator).get();
        outputNames[i] = strdup(outputName.c_str());
    }
}

template<typename inputType, typename outputType> void OnnxHelper<inputType, outputType>::setTensorsShapes()
{
    inputShape = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (inputShape[0] == -1)  inputShape[0] = 1;
    outputShape = session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (outputShape[0] == -1)  outputShape[0] = 1;

    inputSize = 1;
    for (auto e : inputShape)
        inputSize *= e;
    outputSize = 1;
    for (auto e : outputShape)
        outputSize *= e;
}

template<typename inputType, typename outputType> OnnxHelper<inputType, outputType>::OnnxHelper(std::string path)
{
    env = Ort::Env{ORT_LOGGING_LEVEL_ERROR, "Default"};
    session = std::make_unique<Ort::Session>(env, path.c_str(), Ort::SessionOptions{nullptr});

    setTensorsShapes();

    setTensorsNames();
}

template<typename inputType, typename outputType> void OnnxHelper<inputType, outputType>::infer(void* data, void* outputBuffer)
{
    Ort::Value inputTensor = Ort::Value::CreateTensor<inputType>(memoryInfo, (inputType*)data, inputSize, inputShape.data(), inputShape.size());
    Ort::Value outputTensor = Ort::Value::CreateTensor<outputType>(memoryInfo, (outputType*)outputBuffer, outputSize, outputShape.data(), outputShape.size());

    session->Run(Ort::RunOptions{nullptr}, inputNames.get(), &inputTensor, 1, outputNames.get(), &outputTensor, 1);
}