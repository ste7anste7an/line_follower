#pragma once
#include <vector>
#include "UartRemote.h"

std::vector<MRArg>
app_handler(const String& cmd,
            const std::vector<MRArg>& args);