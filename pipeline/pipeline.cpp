#include "pipeline.h"

namespace pipeline
{
    void Pipeline::SetProgramCounter(ui32 pc) 
    {
        programCounter.ChangeInput(pc);
        programCounter.Tact();
    }

    void Pipeline::Tact()
    {
        // Use backward computation of registers' values not to lose previos state
        DoMemory();
        DoExecute();
        DoDecode();
        DoFetch();

        if (!programCounter.GetWriteEnable())
        {
            // Use clear if conflict LW instr
            memoryRegister.Clear();
        }

        RegistersTact();
        programCounter.Tact();
        HazardUnitTact();

        ++tactsNum;
        // DEBUG_LOG((*this));
    }

    void Pipeline::Run()
    {
        try
        {
            while (true)
            {
                Tact();
            }
        }
        catch (std::logic_error& e)
        {}
    }

    void Pipeline::DoFetch()
    {
        // Chech that instructions are ended
        if (!exitInstructionFlag && instrMem.IsPcOutOfSection(programCounter.GetOutput()) &&
            !(executeRegister.GetInput().jmp_cond || executeRegister.GetInput().brn_cond))
        {
            exitInstructionFlag = true;
            if (memoryRegister.GetOutput().jmp_cond)
            {
                // Jump case
                instructionsBeforeExit = 1;
                programCounter.ChangeWE(false);
                decodeRegister.ChangeWE(false);
                executeRegister.ChangeWE(false);
                memoryRegister.ChangeWE(false);
            }
            else if (!pc_r)
            {
                // Other instr case, should finish it
                instructionsBeforeExit = 3;
                programCounter.ChangeWE(false);
                decodeRegister.ChangeWE(false);
                decodeRegister.Clear();
            }
            else
            {
                // Finish execution
                throw std::logic_error("Ended execution");
            }
        }

        instrMem.GetAdress() = programCounter.GetOutput();
        auto first = modules::MUX<ui32>(std::vector<ui32>{programCounter.GetOutput(),
                                            executeRegister.GetOutput().pc_de},
                                            pc_r);
        auto second = modules::MUX<ui32>(std::vector<ui32>{static_cast<ui32>(4), pc_disp}, pc_r);
        programCounter.GetInput() = modules::Add<ui32>(first, second);

        // Fill register
        decodeRegister.GetInput().instr = instrMem.GetInstuction();
        decodeRegister.GetInput().pc_de = programCounter.GetOutput();

        if (exitInstructionFlag)
        {
            if (instructionsBeforeExit)
            {
                --instructionsBeforeExit;
            }
            else
            {
                // Finish execution
                throw std::logic_error("Ended execution");
            }
        }
    }

    void Pipeline::DoDecode()
    {
        auto decodeRegisterOutput = decodeRegister.GetOutput();

        registerFile.ChangeInput(wBRegister.GetOutput().wb_we,
                                 wBRegister.GetOutput().wb_d,
                                 instructions::GetRs1(decodeRegisterOutput.instr),
                                 instructions::GetRs2(decodeRegisterOutput.instr),
                                 wBRegister.GetOutput().wb_a);
        registerFile.Tact();
        controlUnit.ChangeInput(decodeRegisterOutput.instr);
        controlUnit.Tact();

        // Fill register
        executeRegister.GetInput().alu_op = controlUnit.GetALUControl();
        executeRegister.GetInput().alu_src2 = controlUnit.GetAluSrc2();
        executeRegister.GetInput().is_jalr = controlUnit.GetIsJalr();
        executeRegister.GetInput().wb_we = controlUnit.GetWbWE();
        executeRegister.GetInput().mem_we = controlUnit.GetMemWE();
        executeRegister.GetInput().mem_to_reg = controlUnit.GetMemToReg();
        executeRegister.GetInput().brn_cond = controlUnit.GetBrnCond();
        executeRegister.GetInput().jmp_cond = controlUnit.GetJmpCond();
        executeRegister.GetInput().cmp_control = controlUnit.GetCMPControl();
        executeRegister.GetInput().funct3 = controlUnit.GetFunct3();
        executeRegister.GetInput().data1 = modules::MUX<ui32>(std::vector<ui32>{registerFile.GetData1(), decodeRegisterOutput.pc_de},
                                                                (controlUnit.GetOpcode() == 0b0010111));
        executeRegister.GetInput().data1 = modules::MUX<ui32>(std::vector<ui32>{executeRegister.GetInput().data1, 0},
                                                                (controlUnit.GetOpcode() == 0b0110111));
        executeRegister.GetInput().data2 = registerFile.GetData2();
        executeRegister.GetInput().pc_de = decodeRegisterOutput.pc_de;
        executeRegister.GetInput().instr = decodeRegisterOutput.instr;
        executeRegister.GetInput().v_de = modules::Or<bool>(pc_r, decodeRegisterOutput.v_de);
    }

    void Pipeline::DoExecute()
    {
        auto executeRegisterOutput = executeRegister.GetOutput();
        auto src_a = modules::MUX<ui32>(std::vector<ui32>{executeRegisterOutput.data1,
                                         bp_mem,
                                         wBRegister.GetOutput().wb_d},
                                         static_cast<ui8>(hu_rs1));

        auto rs2v = modules::MUX<ui32>(std::vector<ui32>{executeRegisterOutput.data2,
                                                           bp_mem,
                                                           wBRegister.GetOutput().wb_d},
                                                           static_cast<ui8>(hu_rs2));
        pc_disp = instructions::ImmediateExtensionBlock(executeRegisterOutput.instr);
        auto src_b = modules::MUX<ui32>(std::vector<ui32>{rs2v,pc_disp}, executeRegisterOutput.alu_src2);
        bool cmp_res = modules::Cmp<ui32>(src_a, rs2v, executeRegisterOutput.cmp_control);
        pc_r = modules::Or<bool>(executeRegisterOutput.jmp_cond,
                                 modules::And<bool>(cmp_res, executeRegisterOutput.brn_cond));

        // Fill register
        memoryRegister.GetInput().store_mode = executeRegisterOutput.funct3;
        memoryRegister.GetInput().mem_to_reg = executeRegisterOutput.mem_to_reg;
        // WE_GEN
        memoryRegister.GetInput().mem_we = executeRegisterOutput.mem_we && (!executeRegisterOutput.v_de);
        memoryRegister.GetInput().wb_we = executeRegisterOutput.wb_we && (!executeRegisterOutput.v_de);
        // ------
        memoryRegister.GetInput().jmp_cond = executeRegisterOutput.jmp_cond;
        memoryRegister.GetInput().write_data = rs2v;
        memoryRegister.GetInput().alu_res = modules::ALU(src_a, src_b, executeRegisterOutput.alu_op);
        memoryRegister.GetInput().wb_a = instructions::GetRd(executeRegisterOutput.instr);

        if (executeRegisterOutput.is_jalr)
        {
            // Jalr instr case
            pc_disp = memoryRegister.GetInput().alu_res;
            pc_disp &= 0xfffffffe;
        }
        if (executeRegisterOutput.jmp_cond)
        {
            // Jal Jalr instr case
            memoryRegister.GetInput().alu_res = modules::Add<ui32>(executeRegisterOutput.pc_de, 4);
        }
        decodeRegister.GetInput().v_de = pc_r;
    }

    void Pipeline::DoMemory()
    {
        auto memoryRegisterOutput = memoryRegister.GetOutput();
        if (!memoryRegisterOutput.mem_to_reg || memoryRegisterOutput.mem_we)
        {
            dataMem.ChangeInput(memoryRegisterOutput.mem_we, memoryRegisterOutput.alu_res, memoryRegisterOutput.write_data);
            dataMem.GetStoreMode() = memoryRegisterOutput.store_mode;
            dataMem.Tact();
        }
        // Fill register
        wBRegister.GetInput().wb_we = memoryRegisterOutput.wb_we;
        wBRegister.GetInput().wb_d = modules::MUX<ui32>(std::vector<ui32>{dataMem.GetData(),
                                                                  memoryRegisterOutput.alu_res}, 
                                                                  memoryRegisterOutput.mem_to_reg);
                                   
        wBRegister.GetInput().wb_a = memoryRegisterOutput.wb_a;
        bp_mem = memoryRegisterOutput.alu_res;
    }

    void Pipeline::HazardUnitTact()
    {
        if (pc_r)
        {
            // jump or branch. Must clear invalid stages
            decodeRegister.Clear();
            executeRegister.Clear();
            return;
        }

        auto executeRegisterOutput = executeRegister.GetOutput().instr;
        auto instr_type = controlUnit.GetInstructionType();
        auto rs1 = instructions::GetRs1(executeRegisterOutput);
        auto rs2 = instructions::GetRs2(executeRegisterOutput);
        auto mem_wb_a = memoryRegister.GetOutput().wb_a;

        bool stopped = false;
        if (instructions::HasRs1(instr_type) && rs1 != 0)
        {
            if (rs1 == mem_wb_a)
            {
                hu_rs1 = modules::BypassOptionsEncoding::MEM;
                if (!memoryRegister.GetOutput().mem_to_reg)
                {
                    // LW conflict case
                    StopCPU();
                    stopped = true;
                }
            }
            else if (rs1 == wBRegister.GetOutput().wb_a)
            {
                hu_rs1 = modules::BypassOptionsEncoding::WB;
            }
            else
            {
                hu_rs1 = modules::BypassOptionsEncoding::REG;
            }
        }
        else
        {
            hu_rs1 = modules::BypassOptionsEncoding::REG;
        }

        if (instructions::HasRs2(instr_type) && rs2 != 0)
        {
            if (rs2 == mem_wb_a)
            {
                hu_rs2 = modules::BypassOptionsEncoding::MEM;
                if (!memoryRegister.GetOutput().mem_to_reg)
                {
                    // LW conflict case
                    StopCPU();
                    stopped = true;
                }
            }
            else if (rs2 == wBRegister.GetOutput().wb_a)
            {
                hu_rs2 = modules::BypassOptionsEncoding::WB;
            }
            else
            {
                hu_rs2 = modules::BypassOptionsEncoding::REG;
            }
        }
        else
        {
            hu_rs2 = modules::BypassOptionsEncoding::REG;
        }
        if (!stopped)
        {
            // Changes the registers' state only if the pipeline was stalled before;
            RestartPipeline();
        }
    }

    void Pipeline::StopCPU()
    {
        programCounter.ChangeWE(false);
        decodeRegister.ChangeWE(false);
        executeRegister.ChangeWE(false);
    }

    void Pipeline::RestartPipeline()
    {
        programCounter.ChangeWE(true);
        decodeRegister.ChangeWE(true);
        executeRegister.ChangeWE(true);
    }

    void Pipeline::RegistersTact()
    {
        decodeRegister.Tact();
        executeRegister.Tact();
        memoryRegister.Tact();
        wBRegister.Tact();
    }

    void Pipeline::debug()
    {
        DEBUG_LOG(programCounter);
        DEBUG_LOG(decodeRegister);
        DEBUG_LOG(executeRegister);
        DEBUG_LOG(memoryRegister);
        DEBUG_LOG(wBRegister);

        std::cout << "pc_redirect == " << pc_r << std::endl;
        std::cout << "hu_rs1 == " << hu_rs1 << std::endl;
        std::cout << "hu_rs2 == " << hu_rs2 << std::endl;

        DEBUG_LOG(instrMem);
        DEBUG_LOG(dataMem);
        DEBUG_LOG(registerFile);
        DEBUG_LOG(controlUnit);

        PrintRegisters();

        std::cout << "\n\n";
    }
}