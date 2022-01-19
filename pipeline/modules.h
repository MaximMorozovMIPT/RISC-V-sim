#ifndef MODULES_H
#define MODULES_H

#include <iostream>
#include <vector>
#include <exception>
#include <array>
#include <algorithm>
#include <unordered_map>

#include "instructions.h"

namespace modules
{
    using types::ui8;
    using types::ui32;
    using instructions::InstructionType;

    // Bit logic
    template <typename T>
    T Or(T first, T second)
    {
        return first | second;
    }

    template <typename T>
    T And(T first, T second)
    {
        return first & second;
    }

    template <typename T>
    T Add(T first, T second)
    {
        return first + second;
    }

    // Multiplexer
    template <typename T>
    T MUX(std::vector<T> input, ui8 selector)
    {
        if(input.size() <= selector)
        {
            throw std::logic_error("Selector value more than size of input vector in MUX");
        }
        return input[selector];
    }

    // Comparator
    enum class CmpControl : ui8 
    {
        NOP,
        BEQ,
        BNE,
        BLT,
        BGE,
        BLTU,
        BGEU
    };

    template <typename T>
    T Cmp(T first, T second, CmpControl control)
    {
        switch (control) 
        {
            case CmpControl::NOP:
                return false;
            case CmpControl::BEQ:
                return first == second;
            case CmpControl::BNE:
                return first != second;
            case CmpControl::BLT:
            case CmpControl::BLTU:
                return first < second;
            case CmpControl::BGE:
            case CmpControl::BGEU:
                return first >= second;
            default:
                throw std::logic_error("No such command in CmpControl");
        }
    }

    inline std::ostream& operator<<(std::ostream& os, const CmpControl& cmp_control)
    {
        switch (cmp_control) {
            case CmpControl::NOP:
                os << "NOP";
                break;
            case CmpControl::BEQ:
                os << "EQ";
                break;
            case CmpControl::BNE:
                os << "NE";
                break;
            case CmpControl::BLT:
                os << "LT";
                break;
            case CmpControl::BGE:
                os << "GE";
                break;
            case CmpControl::BLTU:
                os << "LTU";
                break;
            case CmpControl::BGEU:
                os << "GEU";
                break;
        }
        return os;
    }

    // ALU
    enum class ALUControl : ui8 
    {
        NOP,
        ADD,
        SUB,
        SLL,
        SLT,
        SLTU,
        XOR,
        SRL,
        SRA,
        OR,
        AND
    };

    inline ui32 ALU(ui32 first, ui32 second, ALUControl control)
    {
        switch (control)
        {
            case ALUControl::NOP:
                    return 0;
            case ALUControl::ADD:
                return first + second;
            case ALUControl::SUB:
                return first - second;
            case ALUControl::AND:
                return first & second;
            case ALUControl::OR:
                return first | second;
            case ALUControl::XOR:
                return first ^ second;
            case ALUControl::SLL:
                second &= 0x0000001f;
                return first << second;
            case ALUControl::SRL:
                second &= 0x0000001f;
                return first >> second;
            case ALUControl::SRA:
                second &= 0x0000001f;
                return (int32_t)first >> second;
            case ALUControl::SLT:
                return ((int32_t)first < (int32_t)second);
            case ALUControl::SLTU:
                return (first < second);
            default:
                throw std::logic_error("No such command in ALUControl " + std::to_string(static_cast<ui8>(control)));
        }
    }

    inline std::ostream& operator<<(std::ostream& os, const ALUControl& alu_control)
    {
        switch (alu_control) {
            case ALUControl::NOP:
                os << "NOP";
                break;
            case ALUControl::ADD:
                os << "ADD";
                break;
            case ALUControl::SUB:
                os << "SUB";
                break;
            case ALUControl::SLL:
                os << "SLL";
                break;
            case ALUControl::SLT:
                os << "SLT";
                break;
            case ALUControl::SLTU:
                os << "SLTU";
                break;
            case ALUControl::XOR:
                os << "XOR";
                break;
            case ALUControl::SRL:
                os << "SRL";
                break;
            case ALUControl::SRA:
                os << "SRA";
                break;
            case ALUControl::OR:
                os << "OR";
                break;
            case ALUControl::AND:
                os << "AND";
                break;
        }
        return os;
    }

    // IMEM
    class InstructionMemory
    {
    public:
        explicit InstructionMemory(const std::vector<ui32>& instructions, ui32 startAddress): address(0)
        {
            std::copy(instructions.cbegin(), instructions.cend(),
                      memory.begin() + startAddress / sizeof(ui32));
            startSection = startAddress;
            endSection = startAddress + instructions.size() * sizeof(ui32);
        }

        ui32 GetInstuction()
        {
            return *reinterpret_cast<ui32 *>(reinterpret_cast<ui8 *>(memory.data()) + address);
        }

        bool IsPcOutOfSection(ui32 pc) const
        {
            return (pc < startSection || pc >= endSection);
        }

        void debug() {
            std::cout << "InstrMemUnit: read_data=" << GetInstuction() << "; address=" << address << std::endl;
        }

        ui32& GetAdress()
        {
            return address;
        }

    private:
        static constexpr size_t capacity = 1024;
        std::array<ui32, capacity> memory = {0};
        ui32 startSection = 0;
        ui32 endSection = 0;
        ui32 address = 0;
    };

    // DMEM
    enum class StoreMode : ui8
    {
        Byte,
        Halfword,
        Word
    };

    inline std::ostream& operator<<(std::ostream& os, const StoreMode& storeMode)
    {
        switch (storeMode) {
            case StoreMode::Byte:
                os << "Byte";
                break;
            case StoreMode::Halfword:
                os << "Halfword";
                break;
            case StoreMode::Word:
                os << "Word";
                break;
        }
        return os;
    }

    enum class LoadExtensionMode : ui8 
    {
        Byte = 0,
        Halfword = 1,
        Word = 2,
        ByteU = 4,
        HalfwordU = 5
    };

    inline std::ostream& operator<<(std::ostream& os, const LoadExtensionMode& load_mode)
    {
        switch (load_mode) {
            case LoadExtensionMode::Byte:
                os << "Byte";
                break;
            case LoadExtensionMode::Halfword:
                os << "Halfword";
                break;
            case LoadExtensionMode::Word:
                os << "Word";
                break;
            case LoadExtensionMode::ByteU:
                os << "ByteU";
                break;
            case LoadExtensionMode::HalfwordU:
                os << "HalfwordU";
                break;
        }
        return os;
    }

    constexpr inline ui32 SignExtendFromByte(ui8 target) {
        if (target >> 7) {
            return static_cast<ui32>(target) | 0xffffff00;
        }
        return static_cast<ui32>(target);
    }

    constexpr inline ui32 SignExtendFromByte2(uint16_t target) {
        if (target >> 15) {
            return static_cast<ui32>(target) | 0xffff0000;
        }
        return static_cast<ui32>(target);
    }

    class DataMemory
    {
    public:
        DataMemory() = default;
        explicit DataMemory(const std::unordered_map<ui32, ui32>& data)
        {
            for (const auto& i : data) {
                *reinterpret_cast<ui32 *>(reinterpret_cast<ui8 *>(memory.data()) + i.first) = i.second;
            }
        }

        void Tact()
        {
            if (writeEnable)
            {
                StoreValue();
            }
            else
            {
                readData = LoadValue();
            }
        }
        
        // Call after Tact if (writeEnable == 0)
        ui32 GetData()
        {
            return LoadValue();
        }

        // Call before Tack with (writeEnable == 1)
        void ChangeInput(bool writeEnable_, ui32 address_, ui32 writeData_)
        {
            writeEnable = writeEnable_;
            address = address_;
            writeData = writeData_;
        }

        ui32& GetAddress()
        {
            return address;
        }

        ui8& GetStoreMode()
        {
            return storeMode;
        }

        void debug() const {
            std::cout << "DataMemUnit: read_data=" << readData << "; write_enable=";
            std::cout << writeData << "; address=" << address << "; storeMode=";
        }

    private:

        void StoreValue() {
            auto storeMode_enum = static_cast<StoreMode>(storeMode);
            switch (storeMode_enum) {
                case StoreMode::Byte:
                    *(reinterpret_cast<ui8 *>(memory.data()) + address) = static_cast<ui8>(writeData);
                    break;
                case StoreMode::Halfword:
                    *reinterpret_cast<uint16_t *>(reinterpret_cast<ui8 *>(memory.data()) + address) = static_cast<uint16_t>(writeData);
                    break;
                case StoreMode::Word:
                    *reinterpret_cast<ui32 *>(reinterpret_cast<ui8 *>(memory.data()) + address) = writeData;
                    break;
            }
        }

        ui32 LoadValue() {
            auto load_mode = static_cast<LoadExtensionMode>(storeMode);
            switch (load_mode) {
                case LoadExtensionMode::Byte:
                    return SignExtendFromByte(*(reinterpret_cast<ui8 *>(memory.data()) + address));
                case LoadExtensionMode::Halfword:
                    return SignExtendFromByte2(*reinterpret_cast<uint16_t *>(reinterpret_cast<ui8 *>(memory.data()) + address));
                case LoadExtensionMode::Word:
                    return *reinterpret_cast<ui32 *>(reinterpret_cast<ui8 *>(memory.data()) + address);
                case LoadExtensionMode::ByteU:
                    return static_cast<ui32>(*(reinterpret_cast<ui8 *>(memory.data()) + address));
                case LoadExtensionMode::HalfwordU:
                    return static_cast<ui32>(*reinterpret_cast<uint16_t *>(reinterpret_cast<ui8 *>(memory.data()) + address));
            }
            throw std::logic_error("invalid funct3: " + std::to_string(storeMode));
        }

        ui8 storeMode = 0;
        bool writeEnable = false;
        ui32 address = 0;
        ui32 writeData = 0;
        ui32 readData = 0;
        static constexpr size_t capacity = 4096;
        std::array<ui32, capacity> memory = {0};
    };

    // Regfile
    class RegisterFile
    {
    public:
        void Tact()
        {
            if (writeEnable3)
            {
                memory.at(address3) = writeData3;
            }
            readData1 = memory.at(address1);
            readData2 = memory.at(address2);
        }

        // Call before Tack with (writeEnable == 1)
        void ChangeInput(bool writeEnable3_, ui32 writeData3_, ui8 address1_, ui8 address2_, ui8 address3_)
        {
            writeEnable3 = writeEnable3_;
            writeData3 = writeData3_;
            address1 = address1_;
            address2 = address2_;
            address3 = address3_;
        }

        ui32 GetData1() const
        {
            return readData1;
        }

        ui32 GetData2() const
        {
            return readData2;
        }

        ui32 GetRegister(ui32 addr) const {
            return memory.at(addr);
        }

        void PrintRegisters() const
        {
            std::cout << "Current registers file state:" << std::endl;
            for (size_t i = 0; i < capacity; ++i) {
                std::cout << "x" << i << " == " << memory[i] << std::endl;
            }
            std::cout << std::endl;
        }

        virtual void debug() const
        {
            std::cout << "RegFile: read_data1=" << readData1 << "; read_data2=" << readData2;
            std::cout << "; write_enable3=" << writeEnable3 << "; write_data3=" << writeData3;
            std::cout << "; address1=" << static_cast<ui32>(address1) << "; address2=";
            std::cout << static_cast<ui32>(address2) << "; address3=" << static_cast<ui32>(address3) << std::endl;
        }

    private:
        bool writeEnable3 = false;
        ui32 writeData3 = 0;
        ui8 address1 = 0;
        ui8 address2 = 0;
        ui8 address3 = 0;
        ui32 readData1 = 0;
        ui32 readData2 = 0;
        static constexpr size_t capacity = 32;
        std::array<ui32, capacity> memory = {0};
    };

    // Register
    template <typename T>
    class Register
    {
    public:
        void Tact()
        {
            if (writeEnable)
            {
                output = input;
            }
        }

        void ChangeInput(T input_)
        {
            input = input_;
        }

        void ChangeWE(bool writeEnable_)
        {
            writeEnable = writeEnable_;
        }

        T GetOutput() const
        {
            return output;
        }

        T& GetInput()
        {
            return input;
        }

        bool GetWriteEnable() const
        {
            return writeEnable;
        }

        void Clear()
        {
            input = 0;
            output = 0;
        }

        void debug() const
        {
            std::cout << "Register: enable_flag=" << writeEnable << "; next=" << input;
            std::cout << "; current=" << output << std::endl;
        }

    private:
        bool writeEnable = true;
        T input = 0;
        T output = 0;
    };

    // CU control unit for alu and cmp
    class ControlUnit
    {
    public:
        void Tact()
        {
            JalrCheck();
            WriteRegisterEnable();
            WriteMemoryEnable();
            IsImmediate();
            CommandALU();
            BranchCondition();
            JumpCondition();
            CommandCmp();
            RegMemCondition();
        }

        void ChangeInput(ui32 instruction_)
        {
            instruction = instruction_;
            funct7 = instructions::GetFunct7(instruction);
            funct3 = instructions::GetFunct3(instruction);
            opcode = instructions::GetOpcode(instruction);
        }

        // Getter methods
        ALUControl GetALUControl() const
        {
            return controlALU;
        }

        CmpControl GetCMPControl() const
        {
            return controlCmp;
        }
        
        bool GetWbWE() const
        {
            return wbWE;
        }

        bool GetMemWE() const
        {
            return memWE;
        }

        bool GetAluSrc2() const
        {
            return aluSrc2;
        }

        bool GetBrnCond() const
        {
            return brnCond;
        }

        bool GetJmpCond() const
        {
            return jmpCond;
        }

        bool GetMemToReg() const
        {
            return memToReg;
        }

        bool GetIsJalr() const
        {
            return isJalr;
        }

        ui8 GetFunct3() const
        {
            return funct3;
        }

        ui8 GetOpcode() const
        {
            return opcode;
        }

        instructions::InstructionType GetInstructionType() const
        {
            return instructionType;
        }

        virtual void debug() const
        {
            std::cout << "ControlUnit: opcode=" << static_cast<ui32>(opcode) << "; funct3=";
            std::cout << static_cast<ui32>(funct3) << "; funct7=" << static_cast<ui32>(funct7);
            std::cout << "; wb_we=" << wbWE;
            std::cout << "; mem_to_reg=" << memToReg << "; brn_cond=" << brnCond << "; jmp_cond=" << jmpCond;
            std::cout << "; cmp_cond=" << controlCmp << "; alu_src2=" << aluSrc2 << "; alu_op=" << controlALU << std::endl;
        }

    private:
        void WriteRegisterEnable()
        {
            if ((opcode == 0b0100011) || (opcode == 0b1100011) || (opcode == 0b1110011))
            {
                wbWE = false;
            }
            else
            {
                wbWE = true;
            }
        }

        void WriteMemoryEnable()
        {
            memWE = (opcode == 0b0100011);
        }

        void IsImmediate()
        {
            instructionType = instructions::GetInstructionType(opcode);

            if (instructionType == InstructionType::IType || 
                instructionType == InstructionType::SType ||
                instructionType == InstructionType::UType)
            {
                aluSrc2 = true;
            }
            else
            {
                aluSrc2 = false;
            }
        }

        void CommandALU()
        {
            if ((opcode == 0b0110011 && funct3 == 0 && funct7 == 0) || 
                (opcode == 0b0010011 && funct3 == 0))
            {
                controlALU = ALUControl::ADD;
            }
            else if (opcode == 0b0110011 && funct3 == 0 && funct7 == 0b0100000)
            {
                controlALU = ALUControl::SUB;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b111) ||
                     (opcode == 0b0010011 && funct3 == 0b111))
            {
                controlALU = ALUControl::AND;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b110) ||
                     (opcode == 0b0010011 && funct3 == 0b110))
            {
                controlALU = ALUControl::OR;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b100) ||
                     (opcode == 0b0010011 && funct3 == 0b100))
            {
                controlALU = ALUControl::XOR;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b001) ||
                     (opcode == 0b0010011 && funct3 == 0b001 && funct7 == 0))
            {
                controlALU = ALUControl::SLL;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b101 && funct7 == 0) ||
                     (opcode == 0b0010011 && funct3 == 0b101 && funct7 == 0))
            {
                controlALU = ALUControl::SRL;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b101 && funct7 == 0b0100000) ||
                     (opcode == 0b0010011 && funct3 == 0b101 && funct7 == 0b0100000))
            {
                controlALU = ALUControl::SRA;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b010) ||
                     (opcode == 0b0010011 && funct3 == 0b010))
            {
                controlALU = ALUControl::SLT;
            }
            else if ((opcode == 0b0110011 && funct3 == 0b011) ||
                     (opcode == 0b0010011 && funct3 == 0b011))
            {
                controlALU = ALUControl::SLTU;
            }
            else if (instructionType == InstructionType::UType || opcode == 0b1100111 ||
                     opcode == 0b0000011 || opcode == 0b0100011) // Last 3 use ADD cause of counting address
            {
                controlALU = ALUControl::ADD;
            }
            else
            {
                controlALU = ALUControl::NOP;
            }
        }

        void BranchCondition()
        {
            brnCond = (opcode == 0b1100011);
        }

        void JumpCondition()
        {
            if (opcode == 0b1101111 || opcode == 0b1100111) {
                jmpCond = true;
            } else {
                jmpCond = false;
            }
        }

        void CommandCmp()
        {
            if (opcode == 0b1100011)
            {
                switch (funct3)
                {
                    case 0b000:
                        controlCmp = CmpControl::BEQ;
                        break;
                    case 0b001:
                        controlCmp = CmpControl::BNE;
                        break;
                    case 0b100:
                        controlCmp = CmpControl::BLT;
                        break;
                    case 0b101:
                        controlCmp = CmpControl::BGE;
                        break;
                    case 0b110:
                        controlCmp = CmpControl::BLTU;
                        break;
                    case 0b111:
                        controlCmp = CmpControl::BGEU;
                        break;
                    default:
                        throw std::logic_error("No such command in CmpControl ControlUnit, funct3 = " + std::to_string(funct3));
                }
            }
            else
            {
                controlCmp = CmpControl::NOP;
            }
        }

        void RegMemCondition()
        {
            memToReg = (opcode != 0b0000011);
        }

        void JalrCheck()
        {
            isJalr = (opcode == 0b1100111);
        }

        ui32 instruction = 0;
        ui8 funct7 = 0;
        ui8 funct3 = 0;
        ui8 opcode = 0;

        ALUControl controlALU = ALUControl::NOP;
        CmpControl controlCmp = CmpControl::BEQ;
        bool wbWE = false;
        bool memWE = false;
        bool aluSrc2 = false;
        bool brnCond = false;
        bool jmpCond = false;
        bool memToReg = false;
        bool isJalr = false;
        instructions::InstructionType instructionType;
    };


    // HU hazard unit
    enum class BypassOptionsEncoding : ui8
    {
        REG,
        MEM,
        WB
    };

    inline std::ostream& operator<<(std::ostream& os, const BypassOptionsEncoding& enc)
    {
        if (enc == BypassOptionsEncoding::REG)
        {
            os << "REG";
        }
        else if (enc == BypassOptionsEncoding::MEM)
        {
            os << "MEM";
        }
        else
        {
            os << "WB";
        }
        return os;
    }
}

#endif