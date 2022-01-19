#include <iostream>
#include <unordered_map>
#include <vector>
#include "../pipeline/pipeline.h"
#include "gtest/gtest.h"


namespace pipeline::tests {

    class InstructionsTest : public ::testing::Test {
    public:
        InstructionsTest() {};

        virtual ~InstructionsTest() {};
    };

    TEST_F(InstructionsTest, AddiTest) {
        const std::vector<pipeline::ui32> instructions = {
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

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;
    }

    TEST_F(InstructionsTest, AddTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x00c58533,     // add  a0, a1, a2
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

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;
    }

    TEST_F(InstructionsTest, RightShiftTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x00402503,     // lw   a0, 4
                0x00300593,     // addi a1, x0, 3
                0x40b55533,     // sra  a0, a0, a1
                0x00000013,     // addi x0, x0, 0 == nop
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00402603,     // lw  a2, 4
                0x00b65633,     // srl a2, a2, a1
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x0000006f      // jr 0
        };
        const pipeline::ui8 data_addr = 4;
        const pipeline::ui32 magic_number = 0x8000ffff;
        std::unordered_map<pipeline::ui32, pipeline::ui32> data = {
                {data_addr, magic_number}
        };

        const pipeline::ui8 reg_addr0 = 10;
        const pipeline::ui32 shift_res0 = 0xf0001fff;
        const pipeline::ui8 reg_addr2 = 12;
        const pipeline::ui32 shift_res2 = 0x10001fff;

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr, data};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        ASSERT_EQ(cpu.GetRegister(reg_addr0), shift_res0);
        ASSERT_EQ(cpu.GetRegister(reg_addr2), shift_res2);
    }

    TEST_F(InstructionsTest, SltiTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x00300593,     // addi a1, x0, 3
                0x0045a513,     // slti a0, a1, 4
                0x00000013,     // addi x0, x0, 0 == nop
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x0000006f      // jr 0
        };

        const pipeline::ui8 reg_addr0 = 10;
        const pipeline::ui8 reg_addr1 = 11;
        const pipeline::ui32 a1_value = 3;
        const pipeline::ui32 cmp_res = 1;

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        ASSERT_EQ(cpu.GetRegister(reg_addr1), a1_value);
        ASSERT_EQ(cpu.GetRegister(reg_addr0), cmp_res);
    }

    TEST_F(InstructionsTest, AuipcTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x00000517,     // auipc a0, 0
                0x00000013,     // addi  x0, x0, 0 == nop
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x0000006f      // jr 0
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const pipeline::ui8 reg_addr = 10;
        ASSERT_EQ(cpu.GetRegister(reg_addr), start_inst_addr);
    }

    TEST_F(InstructionsTest, LwTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x00402503,     // lw   a0, 4
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
        const pipeline::ui32 magic_number = 42;
        std::unordered_map<pipeline::ui32, pipeline::ui32> data = {
                {4, magic_number}
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr, data};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        ASSERT_EQ(cpu.GetRegister(10), magic_number);
    }

    TEST_F(InstructionsTest, LbTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x00400503,     // lb   a0, 4
                0x00404583,     // lbu  a1, 4
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
        const pipeline::ui32 magic_number = 0xffffffff;
        std::unordered_map<pipeline::ui32, pipeline::ui32> data = {
                {4, magic_number}
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr, data};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        ASSERT_EQ(cpu.GetRegister(10), magic_number);
        const pipeline::ui32 zero_extended = 0xff;
        ASSERT_EQ(cpu.GetRegister(11), zero_extended);
    }

    TEST_F(InstructionsTest, SwTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x02a50513,     // addi a0, a0, 42
                0x00000013,     // addi x0, x0, 0 == nop
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00a02223,     // sw a0, x0, 4
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x0000006f      // jr 0
        };

        const pipeline::ui8 reg_addr = 10;
        const pipeline::ui32 magic_number = 42;
        const pipeline::ui32 data_addr = 4;

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        ASSERT_EQ(cpu.GetRegister(reg_addr), magic_number);
        ASSERT_EQ(cpu.GetData(data_addr), magic_number);
    }

    TEST_F(InstructionsTest, SbTest) {
        const std::vector<pipeline::ui32> instructions = {
                0x10150513,     // addi a0, a0, 257
                0x00000013,     // addi x0, x0, 0 == nop
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00a00223,     // sb a0, x0, 4
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x0000006f      // jr 0
        };

        const pipeline::ui8 reg_addr = 10;
        const pipeline::ui32 magic_number = 257;
        const pipeline::ui32 cut_magic_number = 1;
        const pipeline::ui32 data_addr = 4;

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        ASSERT_EQ(cpu.GetRegister(reg_addr), magic_number);
        ASSERT_EQ(cpu.GetData(data_addr), cut_magic_number);
    }
}