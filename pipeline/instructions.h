#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <iostream>
#include "types.h"

namespace instructions 
{
    using types::ui8;
    using types::ui32;

    enum class InstructionType : ui8 
    {
        UNKNOWN,
        RType,
        IType,
        SType,
        BType,
        UType,
        JType
    };

    constexpr ui8 GetFunct7(ui32 instruction)
    {
        return static_cast<ui8>((instruction >> 25) & 0b1111111);
    }

    constexpr ui8 GetRs2(ui32 instruction)
    {
        return static_cast<ui8>((instruction >> 20) & 0b11111);
    }

    constexpr ui8 GetRs1(ui32 instruction)
    {
        return static_cast<ui8>((instruction >> 15) & 0b11111);
    }

    constexpr bool HasRs1(InstructionType type) {
        return (type != InstructionType::UType && type != InstructionType::JType);
    }

    constexpr bool HasRs2(InstructionType type) {
        return (type == InstructionType::RType || type == InstructionType::IType ||
                type == InstructionType::SType || type == InstructionType::BType);
    }

    constexpr ui8 GetFunct3(ui32 instruction) 
    {
        return static_cast<ui8>((instruction >> 12) & 0b111);
    }

    constexpr ui8 GetRd(ui32 instruction)
    {
        return static_cast<ui8>((instruction >> 7) & 0b11111);
    }

    constexpr ui8 GetOpcode(ui32 instruction)
    {
        return static_cast<ui8>(instruction & 0b1111111);
    }

    constexpr InstructionType GetInstructionType(ui8 opcode)
    {
        switch (opcode) {
            case 0b0110011:
                return InstructionType::RType;
            case 0b0100011:
                return InstructionType::SType;
            case 0b1100011:
                return InstructionType::BType;
            case 0b0000011:
            case 0b0010011:
            case 0b1110011:
            case 0b1100111:
                return InstructionType::IType;
            case 0b0110111:
            case 0b0010111:
                return InstructionType::UType;
            case 0b1101111:
                return InstructionType::JType;
            default:
                return InstructionType::UNKNOWN;
        }
    }

    constexpr ui32 ImmediateExtensionBlock(ui32 instruction)
    {
        InstructionType instuctionType = GetInstructionType(GetOpcode(instruction));
        if (instuctionType == InstructionType::UNKNOWN)
        {
            return 0;
        }

        union instructionFormat
        {
            ui32 instruction;

            struct
            {
                ui32 pad0 : 20;
                ui32 imm0 : 12;
            } __attribute__((packed)) instructionIType;

            struct
            {
                ui32 pad0 : 7;
                ui32 imm0 : 5;
                ui32 pad1 : 13;
                ui32 imm1 : 7;
            } __attribute__((packed)) instructionSType;

            struct
            {
                ui32 pad0 : 7;
                ui32 imm2 : 1;
                ui32 imm0 : 4;
                ui32 pad2 : 13;
                ui32 imm1 : 6;
                ui32 sign : 1;
            } __attribute__((packed)) instructionBType;

            struct
            {
                ui32 pad0 : 12;
                ui32 imm0 : 20;
            } __attribute__((packed)) instructionUType;

            struct
            {
                ui32 pad0 : 12;
                ui32 imm2 : 8;
                ui32 imm1 : 1;
                ui32 imm0 : 10;
                ui32 sign : 1;
            } __attribute__((packed)) instructionJType;

        } input{.instruction = instruction};

        union immediateFormat
        {
            ui32 immediate;

            struct
            {
                ui32 imm0 : 12;
                ui32 se : 20;
            } __attribute__((packed)) immediateIType;

            struct
            {
                ui32 imm0 : 5;
                ui32 imm1 : 7;
                ui32 se : 20;
            } __attribute__((packed)) immediateSType;

            struct
            {
                ui32 zero : 1;
                ui32 imm0 : 4;
                ui32 imm1 : 6;
                ui32 imm2 : 1;
                ui32 se : 20;
            } __attribute__((packed)) immediateBType;

            struct
            {
                ui32 zeros : 12;
                ui32 imm0 : 20;
            } __attribute__((packed)) immediateUType;

            struct
            {
                ui32 zero : 1;
                ui32 imm0 : 10;
                ui32 imm1 : 1;
                ui32 imm2 : 8;
                ui32 se : 12;
            } __attribute__((packed)) immediateJType;

        } output{.immediate = 0};

        bool sign = (input.instruction >> 31) & 1;

        switch (instuctionType)
        {
            case InstructionType::RType:
                break;
            case InstructionType::IType:
                output.immediateIType.imm0 = input.instructionIType.imm0;
                if (sign)
                {
                    output.immediateIType.se--;
                }
                break;
            case InstructionType::SType:
                output.immediateSType.imm0 = input.instructionSType.imm0;
                output.immediateSType.imm1 = input.instructionSType.imm1;
                if (sign)
                {
                    output.immediateSType.se--;
                }
                break;
            case InstructionType::BType:
                output.immediateBType.imm0 = input.instructionBType.imm0;
                output.immediateBType.imm1 = input.instructionBType.imm1;
                output.immediateBType.imm2 = input.instructionBType.imm2;
                if (sign)
                {
                    output.immediateBType.se--;
                }
                break;
            case InstructionType::JType:
                output.immediateJType.imm0 = input.instructionJType.imm0;
                output.immediateJType.imm1 = input.instructionJType.imm1;
                output.immediateJType.imm2 = input.instructionJType.imm2;
                if (sign)
                {
                    output.immediateJType.se--;
                }
                break;
            case InstructionType::UType:
                output.immediateUType.imm0 = input.instructionUType.imm0;
                break;
            default:
                throw std::logic_error("invalid instruction type: " + std::to_string(static_cast<ui8>(instuctionType)));
        }

        return output.immediate;
    }
}
#endif