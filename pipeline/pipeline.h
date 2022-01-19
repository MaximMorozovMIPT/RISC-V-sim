#ifndef PIPELINE_H
#define PIPELINE_H

#include <array>
#include <vector>
#include "modules.h"

#define LOG_FIELD(obj, field) os << #field << "=" << std::hex << obj.field << ", "

namespace pipeline
{
    using modules::ui8;
    using modules::ui32;

    struct DecodeRegister
    {
        DecodeRegister(ui32 value) : v_de(static_cast<bool>(value)),
                                   pc_de(value),
                                   instr(value) {};

        bool v_de;
        ui32 pc_de;
        ui32 instr;
    };

    inline std::ostream& operator<<(std::ostream& os, const DecodeRegister& decode_state)
    {
        LOG_FIELD(decode_state, v_de);
        LOG_FIELD(decode_state, pc_de);
        LOG_FIELD(decode_state, instr);
        return os;
    }

    struct ExecuteRegister
    {
        ExecuteRegister(ui32 value) : funct3(static_cast<ui8>(value)),
                                    is_jalr(static_cast<bool>(value)),
                                    alu_op(static_cast<modules::ALUControl>(value)),
                                    alu_src2(static_cast<bool>(value)),
                                    mem_to_reg(static_cast<bool>(value)),
                                    wb_we(static_cast<bool>(value)),
                                    mem_we(static_cast<bool>(value)),
                                    cmp_control(static_cast<modules::CmpControl>(value)),
                                    brn_cond(static_cast<bool>(value)),
                                    jmp_cond(static_cast<bool>(value)),
                                    v_de(static_cast<bool>(value)),
                                    data1(value),
                                    data2(value),
                                    pc_de(value),
                                    instr(value) {};

        ui8 funct3;
        bool is_jalr;
        modules::ALUControl alu_op;
        bool alu_src2;
        bool mem_to_reg;
        bool wb_we;
        bool mem_we;
        modules::CmpControl cmp_control;
        bool brn_cond;
        bool jmp_cond;
        bool v_de;
        ui32 data1;
        ui32 data2;
        ui32 pc_de;
        ui32 instr;
    };

    inline std::ostream& operator<<(std::ostream& os, const ExecuteRegister& execute_state)
    {
        os << "funct3=" << std::hex << static_cast<ui32>(execute_state.funct3) << ", ";
        LOG_FIELD(execute_state, is_jalr);
        LOG_FIELD(execute_state, alu_op);
        LOG_FIELD(execute_state, alu_src2);
        LOG_FIELD(execute_state, mem_to_reg);
        LOG_FIELD(execute_state, wb_we);
        LOG_FIELD(execute_state, mem_we);
        LOG_FIELD(execute_state, cmp_control);
        LOG_FIELD(execute_state, brn_cond);
        LOG_FIELD(execute_state, jmp_cond);
        LOG_FIELD(execute_state, v_de);
        LOG_FIELD(execute_state, data1);
        LOG_FIELD(execute_state, data2);
        LOG_FIELD(execute_state, pc_de);
        LOG_FIELD(execute_state, instr);
        return os;
    }

    struct MemRegister
    {
        MemRegister(ui32 value) : mem_we(static_cast<bool>(value)),
                                   mem_to_reg(static_cast<bool>(value)),
                                   wb_we(static_cast<bool>(value)),
                                   jmp_cond(static_cast<bool>(value)),
                                   store_mode(static_cast<ui8>(value)),
                                   write_data(value),
                                   alu_res(value),
                                   wb_a(value) {};

        bool mem_we;
        bool mem_to_reg;
        bool wb_we;
        bool jmp_cond;
        ui8 store_mode;
        ui32 write_data;
        ui32 alu_res;
        ui32 wb_a;
    };

    inline std::ostream& operator<<(std::ostream& os, const MemRegister& memory_state)
    {
        LOG_FIELD(memory_state, mem_we);
        LOG_FIELD(memory_state, mem_to_reg);
        LOG_FIELD(memory_state, wb_we);
        LOG_FIELD(memory_state, jmp_cond);
        os << "store_mode" << "=" << std::hex << static_cast<ui32>(memory_state.store_mode) << ", ";
        LOG_FIELD(memory_state, write_data);
        LOG_FIELD(memory_state, alu_res);
        LOG_FIELD(memory_state, wb_a);
        return os;
    }

    struct WBRegister
    {
        WBRegister(ui32 value) : wb_we(static_cast<bool>(value)),
                                      wb_d(value),
                                      wb_a(value) {};

        bool wb_we;
        ui32 wb_d;
        ui8 wb_a;
    };

    inline std::ostream& operator<<(std::ostream& os, const WBRegister& wb_state)
    {
        LOG_FIELD(wb_state, wb_we);
        LOG_FIELD(wb_state, wb_d);
        os << "wb_a" << "=" << std::hex << static_cast<ui32>(wb_state.wb_a);
        return os;
    }

    class Pipeline
    {
    public:
        explicit Pipeline(const std::vector<ui32>& instructions, ui32 start_instr_address)
        : instrMem(instructions, start_instr_address) {};

        Pipeline(const std::vector<ui32>& instructions,
                 ui32 start_instr_address,
                 const std::unordered_map<ui32, ui32>& data) : instrMem(instructions, start_instr_address),
                                                                 dataMem(data) {};

        ~Pipeline() noexcept = default;

        void SetProgramCounter(ui32 pc);

        void Tact();

        void Run();

        size_t GetTactsNum() const
        {
            return tactsNum;
        }

        ui32 GetRegister(ui32 addr) const
        {
            return registerFile.GetRegister(addr);
        }

        virtual ui32 GetData(ui32 addr) {
            ui32 res = 0;
            auto current_addr = dataMem.GetAddress();
            dataMem.GetAddress() = addr;
            res = dataMem.GetData();
            dataMem.GetAddress() = current_addr;
            return res;
        }

        void PrintRegisters() const
        {
            registerFile.PrintRegisters();
        }

        void debug();

    private:
        void DoFetch();
        void DoDecode();
        void DoExecute();
        void DoMemory();

        void HazardUnitTact();

        void RegistersTact();
        void StopCPU();
        void RestartPipeline();

        size_t tactsNum = 0;
        int instructionsBeforeExit = 0;
        bool exitInstructionFlag = false;

        ui32 bp_mem = 0;
        ui32 pc_disp = 0;
        bool pc_r = false;

        modules::BypassOptionsEncoding hu_rs1 = modules::BypassOptionsEncoding::REG;
        modules::BypassOptionsEncoding hu_rs2 = modules::BypassOptionsEncoding::REG;

        modules::InstructionMemory instrMem;
        modules::DataMemory dataMem;
        modules::RegisterFile registerFile;
        modules::ControlUnit controlUnit;
        modules::Register<ui32> programCounter;

        modules::Register<DecodeRegister> decodeRegister;
        modules::Register<ExecuteRegister> executeRegister;
        modules::Register<MemRegister> memoryRegister;
        modules::Register<WBRegister> wBRegister;
    };
}

#endif