#include <vector>
#include "pipeline/pipeline.h"

int main()
{
    const std::vector<pipeline::ui32> instructions =
    {
        0x00850513,     // addi a0, a0, 8
        0x00000013,     // addi x0, x0, 0 == nop
        0x00000013,
        0x00000013,
        0x00000013,
        0x00000013,
        0x00000013,
        0x00000013,
        0x00000013,
        0x00000013,
        0x0000006f      // jr 0
    };

    const pipeline::ui32 startAddress = 16;
    pipeline::Pipeline cpu{instructions, startAddress};
    cpu.SetProgramCounter(startAddress);
    cpu.Run();

    return 0;
}