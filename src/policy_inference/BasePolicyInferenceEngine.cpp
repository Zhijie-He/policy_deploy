#include "policy_inference/BasePolicyInferenceEngine.h"
#include "utility/logger.h"
#include "utility/tools.h"

BasePolicyInferenceEngine::BasePolicyInferenceEngine(torch::Device device)
 :device_(device)
{

}

