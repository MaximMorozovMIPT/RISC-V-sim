#include <iostream>
#include <unordered_map>
#include <vector>
#include "../pipeline/pipeline.h"
#include "gtest/gtest.h"


namespace pipeline::tests {

    class IntegrationTest : public ::testing::Test {
    public:
        IntegrationTest() {};

        virtual ~IntegrationTest() {};
    };

    TEST_F(IntegrationTest, FibonacciTest) {
        /*
         * lui x2, 0x20
         * jal main
         * li a2, 255 == addi a2, x0, 255
         * lui a3, 0x30
         * sb a2, 4(a3)
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * main:
         * li a0, 1
         * li a1, 1
         * li a3, 55
         * loop:
         * add a2, a1, a0
         * mv a0, a1 == addi a0, a1, 0
         * mv a1, a2
         * bne a3, a0, loop
         * ret == jr 0
         */
        const std::vector<pipeline::ui32> instructions = {
               0x00020137,
               0x04c000ef,
               0x0ff00613,
               0x000306b7,
               0x00c68223,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00000013,
               0x00100513,
               0x00100593,
               0x03700693,
               0x00a58633,
               0x00b00533,
               0x00c005b3,
               0xfea69ae3,
               0x00008067
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr0 = 10;
        const ui32 reg_addr1 = 13;
        const ui32 magic_number = 55;
        ASSERT_EQ(cpu.GetRegister(reg_addr0), magic_number);
        ASSERT_EQ(cpu.GetRegister(reg_addr1), magic_number);
    }

    TEST_F(IntegrationTest, HazardTest1) {
        /*
         * addi a1, a1, 15
         * addi a2, a2, 16
         * add  a0, a1, a2
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         * nop
         */
        const std::vector<pipeline::ui32> instructions = {             
                0x00f58593,
                0x01060613,
                0x00c58533,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr0 = 10;
        const ui32 reg_addr1 = 11;
        const ui32 reg_addr2 = 12;
        const ui32 magic_number0 = 31;
        const ui32 magic_number1 = 15;
        const ui32 magic_number2 = 16;
        ASSERT_EQ(cpu.GetRegister(reg_addr1), magic_number1);
        ASSERT_EQ(cpu.GetRegister(reg_addr2), magic_number2);
        ASSERT_EQ(cpu.GetRegister(reg_addr0), magic_number0);
    }

    TEST_F(IntegrationTest, HazardTest2) {
        /*
         * jal test
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * test:
         * addi  a0, a0, 42
         * nop
         * nop
         * nop
         * nop
         * nop
         */
        const std::vector<pipeline::ui32> instructions = {
                0x014000ef,
                0x00850513,
                0x00850513,
                0x00850513,
                0x00850513,
                0x02a50513,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr = 10;
        const ui32 magic_number = 42;
        ASSERT_EQ(cpu.GetRegister(reg_addr), magic_number);
    }

    TEST_F(IntegrationTest, HazardTest3) {
        /*
         * lw    a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 42
         * nop
         * nop
         * nop
         * nop
         * nop
         */
        const std::vector<pipeline::ui32> instructions = {
                0x00802503,
                0x00850513,
                0x00850513,
                0x02a50513,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013
        };
        const ui32 magic_number = 1;
        const ui32 data_addr_offset = 8;
        std::unordered_map<pipeline::ui32, pipeline::ui32> data = {
                {data_addr_offset, magic_number}
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr, data};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr = 10;
        const ui32 res = 59;
        ASSERT_EQ(cpu.GetRegister(reg_addr), res);
    }

    TEST_F(IntegrationTest, HazardTest4) {
        /*
         * beq a6, a7, test
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * test:
         * addi  a0, a0, 8
         * nop
         * nop
         * nop
         * nop
         * nop
         */
        const std::vector<pipeline::ui32> instructions = {
                0x01180a63,
                0x00850513,
                0x00850513,
                0x00850513,
                0x00850513,
                0x00850513,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr = 10;
        const ui32 magic_number = 8;
        ASSERT_EQ(cpu.GetRegister(reg_addr), magic_number);
    }

    TEST_F(IntegrationTest, HazardTest5) {
        // NB! this test also checks that without NOP instructions
        // execution of the last 3 instructions is finished correctly
        /*
         * addi  a6, a6, 11
         * addi  a7, a7, 12
         * blt   a6, a7, test
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * ebreak
         * test:
         * addi  a0, a0, 8
         */
        const std::vector<pipeline::ui32> instructions = {
                0x00b80813,
                0x00c88893,
                0x01184a63,
                0x00850513,
                0x00850513,
                0x00850513,
                0x00100073,
                0x00850513
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr = 10;
        const ui32 magic_number = 8;
        ASSERT_EQ(cpu.GetRegister(reg_addr), magic_number);
    }

    TEST_F(IntegrationTest, HazardTest6) {
        /*
         * addi  a6, a6, 11
         * addi  a7, a7, 12
         * bge   a6, a7, test
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * addi  a0, a0, 8
         * jalr  a6, x0, 8
         * test:
         * addi  a0, a0, 8
         * nop
         * nop
         * nop
         * nop
         * nop
         */
        const std::vector<pipeline::ui32> instructions = {
                0x00b80813,
                0x00c88893,
                0x01185a63,
                0x00850513,
                0x00850513,
                0x00850513,
                0x00800867,
                0x00850513,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013,
                0x00000013
        };

        const pipeline::ui32 start_inst_addr = 16;
        pipeline::Pipeline cpu{instructions, start_inst_addr};
        cpu.SetProgramCounter(start_inst_addr);
        cpu.Run();
        std::cout << "Taken " << std::dec << cpu.GetTactsNum() << " ticks to run the program" << std::endl;

        const ui32 reg_addr0 = 10;
        const ui32 calculation_res = 24;
        const ui32 reg_addr1 = 16;
        const ui32 jalr_pc = start_inst_addr + 7 * sizeof(ui32);
        ASSERT_EQ(cpu.GetRegister(reg_addr0), calculation_res);
        ASSERT_EQ(cpu.GetRegister(reg_addr1), jalr_pc);
    }
}