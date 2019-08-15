

#ifndef ROBOTICAN_CONTROLLERS_UTILS_H
#define ROBOTICAN_CONTROLLERS_UTILS_H

#include <ros/ros.h>
namespace rosUtil {
    void rosInfo(const char *info);

    void rosError(const char *err);

    void rosWarn(const char *warn);

    template <typename T>
    void rosInfoStream(T &obj);

}
#endif //ROBOTICAN_CONTROLLERS_UTILS_H
