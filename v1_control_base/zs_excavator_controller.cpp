#include <windows.h>

#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

class ZSCanTransport {
public:
    ZSCanTransport(const std::string& port = "COM3", uint32_t baudrate = 115200, DWORD timeout_ms = 1000)
        : port_(port), baudrate_(baudrate), timeout_ms_(timeout_ms), handle_(INVALID_HANDLE_VALUE) {}

    ~ZSCanTransport() {
        close();
    }

    bool open() {
        if (isOpen()) {
            return true;
        }

        const std::string device_path = buildDevicePath(port_);
        handle_ = CreateFileA(
            device_path.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );

        if (handle_ == INVALID_HANDLE_VALUE) {
            std::cerr << "[ERROR] Failed to open serial port: " << port_ << std::endl;
            return false;
        }

        DCB dcb = {};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(handle_, &dcb)) {
            std::cerr << "[ERROR] GetCommState failed" << std::endl;
            close();
            return false;
        }

        dcb.BaudRate = baudrate_;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;

        if (!SetCommState(handle_, &dcb)) {
            std::cerr << "[ERROR] SetCommState failed" << std::endl;
            close();
            return false;
        }

        COMMTIMEOUTS timeouts = {};
        timeouts.ReadIntervalTimeout = timeout_ms_;
        timeouts.ReadTotalTimeoutConstant = timeout_ms_;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = timeout_ms_;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        SetCommTimeouts(handle_, &timeouts);

        std::cout << "[INFO] Serial opened: " << port_ << " @ " << baudrate_ << " bps" << std::endl;
        return true;
    }

    void close() {
        if (isOpen()) {
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
            std::cout << "[INFO] Serial closed" << std::endl;
        }
    }

    bool isOpen() const {
        return handle_ != INVALID_HANDLE_VALUE;
    }

    void handshake(int delay_ms = 500) {
        sendCanFrame(0x0303, {0, 0, 0, 0, 0, 0, 0, 0});
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    std::array<uint8_t, 13> sendCanFrame(uint32_t can_id, const std::vector<uint8_t>& data, bool is_extended = false, uint8_t func_code = 0x00) {
        if (data.size() > 8) {
            throw std::runtime_error("CAN data must not exceed 8 bytes");
        }

        if (!isOpen()) {
            throw std::runtime_error("Serial port is not open");
        }

        std::array<uint8_t, 13> frame = {};
        const auto id_bytes = encodeCanId(can_id, is_extended);

        for (size_t i = 0; i < 4; ++i) {
            frame[i] = id_bytes[i];
        }
        for (size_t i = 0; i < data.size(); ++i) {
            frame[4 + i] = data[i];
        }
        frame[12] = func_code;

        DWORD written = 0;
        if (!WriteFile(handle_, frame.data(), static_cast<DWORD>(frame.size()), &written, nullptr) || written != frame.size()) {
            throw std::runtime_error("Failed to write frame to serial port");
        }

        std::cout << "[TX] ID=0x" << std::hex << std::uppercase << can_id
                  << " DATA=" << bytesToHex(std::vector<uint8_t>(frame.begin() + 4, frame.begin() + 12))
                  << " FRAME=" << bytesToHex(std::vector<uint8_t>(frame.begin(), frame.end()))
                  << std::dec << std::endl;
        return frame;
    }

    std::array<uint8_t, 13> sendRaw12ByteCommand(const std::string& hex_command, bool is_extended = false) {
        const auto values = parse12ByteCommand(hex_command);
        const uint32_t can_id =
            (static_cast<uint32_t>(values[0]) << 24) |
            (static_cast<uint32_t>(values[1]) << 16) |
            (static_cast<uint32_t>(values[2]) << 8) |
            static_cast<uint32_t>(values[3]);
        return sendCanFrame(can_id, std::vector<uint8_t>(values.begin() + 4, values.end()), is_extended);
    }

private:
    static std::string buildDevicePath(const std::string& port) {
        if (port.rfind("\\\\.\\", 0) == 0) {
            return port;
        }
        return "\\\\.\\" + port;
    }

    static std::array<uint8_t, 4> encodeCanId(uint32_t can_id, bool is_extended) {
        std::array<uint8_t, 4> bytes = {};
        if (is_extended) {
            uint32_t shifted = (can_id << 3) & 0xFFFFFFFFu;
            bytes[0] = static_cast<uint8_t>((shifted >> 24) & 0xFF);
            bytes[1] = static_cast<uint8_t>((shifted >> 16) & 0xFF);
            bytes[2] = static_cast<uint8_t>((shifted >> 8) & 0xFF);
            bytes[3] = static_cast<uint8_t>(shifted & 0xFF);
            bytes[3] |= 0x02;
            return bytes;
        }

        uint32_t shifted = (can_id << 21) & 0xFFFFFFFFu;
        bytes[0] = static_cast<uint8_t>((shifted >> 24) & 0xFF);
        bytes[1] = static_cast<uint8_t>((shifted >> 16) & 0xFF);
        bytes[2] = static_cast<uint8_t>((shifted >> 8) & 0xFF);
        bytes[3] = static_cast<uint8_t>(shifted & 0xFF);
        return bytes;
    }

    static std::string stripComment(const std::string& input) {
        size_t hash_pos = input.find('#');
        size_t slash_pos = input.find("//");
        size_t end_pos = std::min(
            hash_pos == std::string::npos ? input.size() : hash_pos,
            slash_pos == std::string::npos ? input.size() : slash_pos
        );
        return input.substr(0, end_pos);
    }

    static std::array<uint8_t, 12> parse12ByteCommand(const std::string& hex_command) {
        std::array<uint8_t, 12> values = {};
        std::istringstream stream(stripComment(hex_command));
        std::string token;
        size_t index = 0;

        while (stream >> token) {
            if (index >= values.size()) {
                throw std::runtime_error("Expected exactly 12 bytes in command");
            }
            values[index++] = static_cast<uint8_t>(std::stoul(token, nullptr, 16));
        }

        if (index != values.size()) {
            throw std::runtime_error("Expected exactly 12 bytes in command");
        }
        return values;
    }

    static std::string bytesToHex(const std::vector<uint8_t>& data) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');
        for (size_t i = 0; i < data.size(); ++i) {
            if (i != 0) {
                oss << ' ';
            }
            oss << std::setw(2) << static_cast<int>(data[i]);
        }
        return oss.str();
    }

    std::string port_;
    uint32_t baudrate_;
    DWORD timeout_ms_;
    HANDLE handle_;
};


class ExcavatorController {
public:
    static constexpr uint32_t ID_ARM_SWING = 0x0101;
    static constexpr uint32_t ID_BOOM_BUCKET = 0x0102;
    static constexpr uint32_t ID_CHASSIS = 0x0103;
    static constexpr uint32_t ID_ANALOG = 0x0104;

    explicit ExcavatorController(ZSCanTransport& transport) : transport_(transport) {}

    bool connect(bool do_handshake = true) {
        if (!transport_.open()) {
            return false;
        }
        if (do_handshake) {
            transport_.handshake();
        }
        return true;
    }

    void close() {
        transport_.close();
    }

    void setAnalog(std::optional<uint16_t> ch1_mv = std::nullopt,
                   std::optional<uint16_t> ch2_mv = std::nullopt,
                   std::optional<uint16_t> ch3_mv = std::nullopt) {
        std::vector<uint8_t> data(8, 0x00);
        if (ch1_mv.has_value()) {
            setU16(data, 0, checkMv(*ch1_mv));
        }
        if (ch2_mv.has_value()) {
            setU16(data, 2, checkMv(*ch2_mv));
        }
        if (ch3_mv.has_value()) {
            setU16(data, 4, checkMv(*ch3_mv));
        }
        transport_.sendCanFrame(ID_ANALOG, data);
    }

    void stopAnalog() {
        setAnalog(0, 0, 0);
    }

    void stopChassis() {
        sendSingleByteAction(ID_CHASSIS, 0x00);
    }

    void stopBoomBucket() {
        sendSingleByteAction(ID_BOOM_BUCKET, 0x00);
    }

    void stopArmSwing() {
        sendSingleByteAction(ID_ARM_SWING, 0x00);
    }

    void stopAll() {
        stopChassis();
        stopBoomBucket();
        stopArmSwing();
        stopAnalog();
    }

    void driveForward(uint16_t left_mv, uint16_t right_mv) {
        setAnalog(right_mv, left_mv, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x06);
    }

    void driveBackward(uint16_t left_mv, uint16_t right_mv) {
        setAnalog(right_mv, left_mv, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x09);
    }

    void turnLeft(uint16_t left_mv, uint16_t right_mv) {
        setAnalog(right_mv, left_mv, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x0A);
    }

    void turnRight(uint16_t left_mv, uint16_t right_mv) {
        setAnalog(right_mv, left_mv, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x05);
    }

    void leftTrackForward(uint16_t mv) {
        setAnalog(std::nullopt, mv, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x02);
    }

    void leftTrackBackward(uint16_t mv) {
        setAnalog(std::nullopt, mv, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x01);
    }

    void rightTrackForward(uint16_t mv) {
        setAnalog(mv, std::nullopt, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x04);
    }

    void rightTrackBackward(uint16_t mv) {
        setAnalog(mv, std::nullopt, std::nullopt);
        sendSingleByteAction(ID_CHASSIS, 0x08);
    }

    void boomUp() {
        sendSingleByteAction(ID_BOOM_BUCKET, 0x02);
    }

    void boomDown() {
        sendSingleByteAction(ID_BOOM_BUCKET, 0x01);
    }

    void bucketIn() {
        sendSingleByteAction(ID_BOOM_BUCKET, 0x04);
    }

    void bucketOut() {
        sendSingleByteAction(ID_BOOM_BUCKET, 0x08);
    }

    void armPush() {
        sendSingleByteAction(ID_ARM_SWING, 0x02);
    }

    void armPull() {
        sendSingleByteAction(ID_ARM_SWING, 0x01);
    }

    void swingLeft() {
        sendSingleByteAction(ID_ARM_SWING, 0x08);
    }

    void swingRight() {
        sendSingleByteAction(ID_ARM_SWING, 0x04);
    }

    void sendNamedRaw(const std::string& name) {
        if (name == "forward") {
            transport_.sendRaw12ByteCommand("00 00 01 03 06 00 00 00 00 00 00 00");
        } else if (name == "backward") {
            transport_.sendRaw12ByteCommand("00 00 01 03 09 00 00 00 00 00 00 00");
        } else if (name == "turn_left") {
            transport_.sendRaw12ByteCommand("00 00 01 03 0A 00 00 00 00 00 00 00");
        } else if (name == "turn_right") {
            transport_.sendRaw12ByteCommand("00 00 01 03 05 00 00 00 00 00 00 00");
        } else if (name == "boom_up") {
            transport_.sendRaw12ByteCommand("00 00 01 02 02 00 00 00 00 00 00 00");
        } else if (name == "boom_down") {
            transport_.sendRaw12ByteCommand("00 00 01 02 01 00 00 00 00 00 00 00");
        } else if (name == "bucket_in") {
            transport_.sendRaw12ByteCommand("00 00 01 02 04 00 00 00 00 00 00 00");
        } else if (name == "bucket_out") {
            transport_.sendRaw12ByteCommand("00 00 01 02 08 00 00 00 00 00 00 00");
        } else if (name == "arm_push") {
            transport_.sendRaw12ByteCommand("00 00 01 01 02 00 00 00 00 00 00 00");
        } else if (name == "arm_pull") {
            transport_.sendRaw12ByteCommand("00 00 01 01 01 00 00 00 00 00 00 00");
        } else if (name == "swing_left") {
            transport_.sendRaw12ByteCommand("00 00 01 01 08 00 00 00 00 00 00 00");
        } else if (name == "swing_right") {
            transport_.sendRaw12ByteCommand("00 00 01 01 04 00 00 00 00 00 00 00");
        } else {
            throw std::runtime_error("Unknown raw command name: " + name);
        }
    }

private:
    static uint16_t checkMv(uint16_t value) {
        if (value > 5000) {
            throw std::runtime_error("Analog value must be in range 0..5000");
        }
        return value;
    }

    static void setU16(std::vector<uint8_t>& data, size_t offset, uint16_t value) {
        data[offset] = static_cast<uint8_t>((value >> 8) & 0xFF);
        data[offset + 1] = static_cast<uint8_t>(value & 0xFF);
    }

    void sendSingleByteAction(uint32_t can_id, uint8_t action_code) {
        transport_.sendCanFrame(can_id, {action_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    }

    ZSCanTransport& transport_;
};


#ifdef BUILD_ZS_CONTROLLER_DEMO
int main() {
    ZSCanTransport transport("COM3", 115200, 1000);
    ExcavatorController controller(transport);
    if (!controller.connect()) {
        return 1;
    }

    try {
        controller.driveForward(5000, 5000);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        controller.stopAll();
    } catch (const std::exception& ex) {
        std::cerr << "[ERROR] " << ex.what() << std::endl;
    }

    controller.close();
    return 0;
}
#endif
