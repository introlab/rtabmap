#include <rtabmap/core/OrtInterface.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

OrtInterface::OrtInterface()
{
    switch(ULogger::level())
    {
        case ULogger::kDebug:
            env_ = Ort::Env(ORT_LOGGING_LEVEL_VERBOSE);
            break;
        case ULogger::kInfo:
            env_ = Ort::Env(ORT_LOGGING_LEVEL_INFO);
            break;
        case ULogger::kWarning:
            env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING);
            break;
        case ULogger::kError:
            env_ = Ort::Env(ORT_LOGGING_LEVEL_ERROR);
            break;
        case ULogger::kFatal:
            env_ = Ort::Env(ORT_LOGGING_LEVEL_FATAL);
    }
}

Ort::Session OrtInterface::createSession(const std::string & model_path, const Ort::SessionOptions & options)
{
    return Ort::Session(env_, model_path.c_str(), options);
}

}