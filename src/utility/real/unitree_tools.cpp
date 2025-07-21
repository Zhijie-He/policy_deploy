// utility/tools.h
#include "utility/real/unitree_tools.h"

namespace unitree_tools {
    uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
        uint32_t xbit = 0;
        uint32_t data = 0;
        uint32_t CRC32 = 0xFFFFFFFF;
        const uint32_t dwPolynomial = 0x04c11db7;
        for (uint32_t i = 0; i < len; i++) {
            xbit = 1 << 31;
            data = ptr[i];
            for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;

            xbit >>= 1;
            }
        }
        return CRC32;
    };
    
    void create_zero_cmd(LowCmd_& cmd) {
        size_t size = cmd.motor_cmd().size();
        for (size_t i = 0; i < size; ++i) {
            cmd.motor_cmd()[i].q() = 0.0f;
            cmd.motor_cmd()[i].dq() = 0.0f;
            cmd.motor_cmd()[i].kp() = 0.0f;
            cmd.motor_cmd()[i].kd() = 0.0f;
            cmd.motor_cmd()[i].tau() = 0.0f;
        }
    }

    void init_cmd_hg(LowCmd_& cmd, uint8_t mode_machine, Mode mode_pr) {
        cmd.mode_machine() = mode_machine;
        cmd.mode_pr() = static_cast<uint8_t>(mode_pr);
        
        // here should use num actions
        size_t size = cmd.motor_cmd().size();
        for (size_t i = 0; i < size; ++i) {
            cmd.motor_cmd()[i].mode() = 1;
            cmd.motor_cmd()[i].q() = 0;
            cmd.motor_cmd()[i].dq() = 0;
            cmd.motor_cmd()[i].kp() = 0;
            cmd.motor_cmd()[i].kd() = 0;
            cmd.motor_cmd()[i].tau() = 0;
        }
    }

    void print_lowcmd(const LowCmd_& cmd) {
        std::cout << "=== LowCmd_ ===" << std::endl;
        std::cout << "Mode PR: " << (int)cmd.mode_pr() << std::endl;
        std::cout << "Mode Machine: " << (int)cmd.mode_machine() << std::endl;

        for (size_t i = 0; i < cmd.motor_cmd().size(); ++i) {
            const auto& m = cmd.motor_cmd().at(i);
            std::cout << "Motor[" << i << "]"
                    << "  mode=" << (int)m.mode()
                    << "  tau=" << m.tau()
                    << "  q=" << m.q()
                    << "  dq=" << m.dq()
                    << "  kp=" << m.kp()
                    << "  kd=" << m.kd()
                    << std::endl;
        }
    }


}
