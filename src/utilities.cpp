#include "utilities.h"

namespace ks {

TimePoint (*GetCurrentTime)(void) = &std::chrono::system_clock::now;

}