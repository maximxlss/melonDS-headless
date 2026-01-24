#ifndef MELONDS_HEADLESS_PLATFORM_H
#define MELONDS_HEADLESS_PLATFORM_H

#include "Platform.h"

namespace melonDS::Platform
{
bool Headless_StopRequested();
StopReason Headless_StopReason();
}

#endif
