#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include <iostream>

#define DEBUG_LOG(obj) std::cout << std::dec << __LINE__ << std::hex << ": "; obj.debug()

namespace types
{
    using ui8 = uint8_t;
    using ui32 = uint32_t;
}
#endif