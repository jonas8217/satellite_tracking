#include <stdint.h>
#include "ProtocolRoTxCfg.h"
#include "utils.cpp"

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

#include <chrono>
#include <iostream>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>

#define PORT "/dev/ttyUSB0"
#define BAUDRATE B115200

#define MAX_RETURN_MSG_LEN 43  // specifically in set and get config otherwise ususally 12 bytes, only other exception is GET_OUTS which returns 2 bytes
#define TRANSMIT_MSG_LEN 13
#define NOMINAL_RETURN_MSG_BYTES 12

#define DEBUG_COMMS 0
#define DO_CONTROL 0

#define MICRO_SEC_PER_SEC 1000000.0

int SERIAL_PORT;
struct termios tty_;
bool setup_complete = false;
uint8_t READ_BUF[MAX_RETURN_MSG_LEN];
uint8_t WRITE_BUF[TRANSMIT_MSG_LEN];


int min_power = 18;  // lowest possible power which still moves consistently without stalling
int max_power = 100; // max possible power

enum MSG_TYPE {
    CMD_GET_MOTOR_ANGLES,      // Get current motors positions
    CMD_GET_MOTOR_ANGLES_100,  // Get current motors positions. 0.01 resolution
    CMD_SET_ANGLES,            // Move motors to position.
    CMD_SET_ANGLES_100,        // Move motors to position. 0.01 resolution
    CMD_CFG_GET,               // (has not been setup) Get settings value. isSketchValue determines, if response provides value for current running settings or for prepared settings to be applied in bulk. Passing fieldId = 0 in response returns maximum fiedlId in fieldValue.f_word
    CMD_GET_SOFT_HARD,         // Get START and STOP settinsend_recvgs (IMMEDIATELY/SOFTLY)
    CMD_SET_SOFT_HARD,         // Set start/stop immediately or softly settings.
    CMD_RESTART_DEVICE,        // Restarts device after 5 seconds. Payload restartConfirmValue must be set to: rotxMagicRestartDevice
    CMD_STOP,                  // Stop motors immediately.
    CMD_MOTORS,                // Command motors move (left/right etc.)
    CMD_POWER,                 // Set motors power (0-100%). (Applied immediately, without stoping current move)
} msg_type;

const bool MSG_HAS_INPUT[] = {0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1};

const uint8_t MSG_ARRAYS[][13] = {
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x20},  // CMD_GET_MOTOR_ANGLES
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0x20},  // CMD_GET_MOTOR_ANGLES_100
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x2f, 0x20},  // CMD_SET_ANGLES
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5f, 0x20},  // CMD_SET_ANGLES_100
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xef, 0x20},  // CMD_CFG_GET
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa1, 0x20},  // CMD_GET_SOFT_HARD
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa2, 0x20},  // CMD_SET_SOFT_HARD
    {0x57, 0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee, 0x20},  // CMD_RESTART_DEVICE
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x20},  // CMD_STOP
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x20},  // CMD_MOTORS
    {0x57, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x42, 0xf7, 0x20}   // CMD_POWER
};

int setup_USB_UART_connection() {
    SERIAL_PORT = open(PORT, O_RDWR);

    // Info from this blog
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

    struct termios* tty = &tty_;

    if (SERIAL_PORT < 0) {
        printf("Error %i from open trying to open %s: %s\n", errno, PORT, strerror(errno));
        return -1;
    }

    if (tcgetattr(SERIAL_PORT, tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }
    tty->c_cflag &= ~PARENB;  // No parity bit
    tty->c_cflag &= ~CSTOPB;  // Only one stop bit
    tty->c_cflag &= ~CSIZE;   // Clear all the size bits then set:
    tty->c_cflag |= CS8;      // 8 bits per byte

    tty->c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control
    tty->c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty->c_lflag &= ~ICANON;         // Disable cononical mode (don't process input)

    tty->c_lflag &= ~ECHO;    // Disable echo
    tty->c_lflag &= ~ECHOE;   // Disable erasure
    tty->c_lflag &= ~ECHONL;  // Disable new-line echo

    tty->c_lflag &= ~ISIG;                                                         // Disable interpretation of INTR, QUIT and SUSP
    tty->c_iflag &= ~(IXON | IXOFF | IXANY);                                       // Turn off software flow ctrl
    tty->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);  // Disable any special handling of received bytes

    tty->c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
    tty->c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

    // temporaily set both to 0 to be able clear buffers
    tty->c_cc[VTIME] = 0;
    tty->c_cc[VMIN] = 0;

    // Baudrate is 9600, could possible be made faster TODO
    cfsetispeed(tty, BAUDRATE);
    cfsetospeed(tty, BAUDRATE);

    if (tcsetattr(SERIAL_PORT, TCSANOW, tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    read(SERIAL_PORT, &READ_BUF, MAX_RETURN_MSG_LEN);  // clear buffer
    // Then setup for real configuration

    // Either wait for 12 bytes or for 1 decisecond to stop doing a blocking read, whichever comes first
    tty->c_cc[VTIME] = 1;  // Block for 1 decisecond.
    tty->c_cc[VMIN] = 12;  // Block until 12 bytes are recieved
    // buffer setup for returned commands to allow for fast control-loop

    if (tcsetattr(SERIAL_PORT, TCSANOW, tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    setup_complete = true;
    if (DEBUG_COMMS) {
        printf("USB-UART connection established\n");
    }

    return 0;
}

void send(MSG_TYPE type) {
    if (!setup_complete) {
        printf("Cannot transmit message, setup has not been completed\n");
        return;
    }

    if (!MSG_HAS_INPUT[type]) {
        for (int i = 0; i < TRANSMIT_MSG_LEN; i++) {
            WRITE_BUF[i] = MSG_ARRAYS[type][i];
        }
    }

    if (DEBUG_COMMS) {
        printf("Transmitting: ");
        for (int i = 0; i < TRANSMIT_MSG_LEN; i++) {
            printf("0x%02x ", WRITE_BUF[i]);
        }
        printf("\n");
    }

    write(SERIAL_PORT, WRITE_BUF, TRANSMIT_MSG_LEN);

    // clear WRITE_BUF to make sure no mistakes are made while testing message functions
    for (int i = 0; i < TRANSMIT_MSG_LEN; i++) {
        WRITE_BUF[i] = 0x00;
    }
}

void recv(int expected_return_bytes) {
    int num_bytes = 0;
    if (expected_return_bytes > NOMINAL_RETURN_MSG_BYTES) {
        while (num_bytes < expected_return_bytes) {
            num_bytes += read(SERIAL_PORT, &*(READ_BUF + num_bytes), MAX_RETURN_MSG_LEN);
        }
    } else {
        num_bytes = read(SERIAL_PORT, &READ_BUF, MAX_RETURN_MSG_LEN);
    }

    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
    }
    if (num_bytes < expected_return_bytes) {
        printf("Return message incomplete expected: %i, Recieved: %i\n", expected_return_bytes, num_bytes);
    }

    if (DEBUG_COMMS) {
        printf("Recieved:     ");
        for (int i = 0; i < num_bytes; i++) {
            printf("0x%02x ", READ_BUF[i]);
        }
        printf("\n");
    }
}

void send_recv(MSG_TYPE type, int expected_return_bytes) {
    if (!setup_complete) {
        printf("Cannot transmit message, setup has not been completed\n");
        return;
    }
    send(type);
    recv(expected_return_bytes);
}

void setup_write_buffer_for_input(MSG_TYPE type) {
    for (int i = 0; i < TRANSMIT_MSG_LEN; i++) {
        WRITE_BUF[i] = MSG_ARRAYS[type][i];
    }
}

void print_read_buf(int msg_len = NOMINAL_RETURN_MSG_BYTES) {
    for (int i = 0; i < msg_len; i++) {
        printf("0x%02x ", READ_BUF[i]);
    }
    printf("\n");
}

void print_write_buf(int msg_len = TRANSMIT_MSG_LEN) {
    for (int i = 0; i < msg_len; i++) {
        printf("0x%02x ", WRITE_BUF[i]);
    }
    printf("\n");
}

void basic_message_get_debug(MSG_TYPE type) {
    send_recv(type, NOMINAL_RETURN_MSG_BYTES);
    if (!DEBUG_COMMS) {  // will already be printed if debug is on
        printf("Read bytes:\n");
        for (int i = 0; i < NOMINAL_RETURN_MSG_BYTES; i++) {
            printf("0x%02x ", READ_BUF[i]);
        }
        printf("\n");
    }
}

union {
  float f;
  u_char b[4];
} u_char_float;

struct cfg_value {
    rotCfgDataType data_type = rotCfgDataType::rcdt_NONE;
    int int_ = 0;
    double double_ = 0.0;
    uint8_t bytes[4];
};


cfg_value interpret_cfg_value(int field_id) {
    cfg_value value_struct;

    if (rotCfgFields[field_id].dataType == rotCfgDataType::rcdt_float) {
        for (int i = 0; i < 4; i++) u_char_float.b[i] = (READ_BUF + 0x27)[i];  // Value field
        value_struct.double_ = u_char_float.f;
        value_struct.data_type = rotCfgDataType::rcdt_float;
    } else if (rotCfgFields[field_id].dataType >= rotCfgDataType::rcdt_uint32_t && rotCfgFields[field_id].dataType <= rotCfgDataType::rcdt_uint8_t) {
        int mult = 1;
        for (int i = 0; i < 4; i++) {
            value_struct.int_ += mult * (READ_BUF + 0x27)[i];
            mult *= 16;
        }
        value_struct.data_type = rotCfgDataType::rcdt_uint32_t;
    } else {
        for (int i = 0; i < 4; i++) value_struct.bytes[i] = (READ_BUF + 0x27)[i];  // Value field
        value_struct.data_type = rotCfgDataType::rcdt_NONE;
    }
    return value_struct;
}

void print_cfg() {
    int field_id = READ_BUF[1] + 256 * READ_BUF[2];
    printf("Field: %d\n", field_id);
    printf("Name: %s\n", std::string((char*)READ_BUF + 3, 32).c_str());
    cfg_value cfg_value = interpret_cfg_value(field_id);

    if (cfg_value.data_type == rotCfgDataType::rcdt_float) {
        printf("Value: %.2f \n", cfg_value.double_);
    } else if (cfg_value.data_type == rotCfgDataType::rcdt_uint32_t) {
        printf("Value: %d \n", cfg_value.int_);
    } else {
        printf("Value: 0x%02x 0x%02x 0x%02x 0x%02x \n", cfg_value.bytes[0], cfg_value.bytes[1], cfg_value.bytes[2], cfg_value.bytes[3]);
    }
}

cfg_value cfg_values[sizeof(rotCfgFields)];

cfg_value get_configuration(int field_id = -1) {

    setup_write_buffer_for_input(CMD_CFG_GET);
    if (field_id == -1) {
        for (int i = 0; i < sizeof(rotCfgFields); i++) {  // TODO check sizeof or use "max_field_id" as acquired from the control box
            WRITE_BUF[1] = i % 256;
            WRITE_BUF[2] = i / 256;
            send_recv(CMD_CFG_GET, MAX_RETURN_MSG_LEN);
            cfg_values[i] = interpret_cfg_value(i);
        }
        return cfg_values[0]; // to get rid of a warning
    } else {
        WRITE_BUF[1] = field_id % 256;
        WRITE_BUF[2] = field_id / 256;
        send_recv(CMD_CFG_GET, MAX_RETURN_MSG_LEN);
        if (field_id != READ_BUF[1] + 256 * READ_BUF[2]) {
            printf("recieved field id is not the same as transmitted field id :(");
        }
        return interpret_cfg_value(field_id);
    }
}
void decode_angles(double* angle_output) {
    // angle = StrToInt(receivedAngle) * divisor - 360 * divisor
    int a1 = READ_BUF[1 + 0] * 1000 + READ_BUF[1 + 1] * 100 + READ_BUF[1 + 2] * 10 + READ_BUF[1 + 3] * 1;  // convert to integer
    int a2 = READ_BUF[6 + 0] * 1000 + READ_BUF[6 + 1] * 100 + READ_BUF[6 + 2] * 10 + READ_BUF[6 + 3] * 1;
    angle_output[0] = a1 / (double)READ_BUF[5] - 360.0;  // do the math
    angle_output[1] = a2 / (double)READ_BUF[10] - 360.0;
}

void get_angles(double* angle_output) {
    send_recv(CMD_GET_MOTOR_ANGLES, NOMINAL_RETURN_MSG_BYTES);

    decode_angles(angle_output);
}


void get_angles_100(double* angle_output) {  // unlike in CMD_GET_MOTOR_ANGLES the devisor is set to a constant 100, this makes room for one more byte of data for the angles

    send_recv(CMD_GET_MOTOR_ANGLES_100, NOMINAL_RETURN_MSG_BYTES);

    // angle = StrToInt(receivedAngle) * divisor - 360 * divisor
    int a1 = READ_BUF[1 + 0] * 10000 + READ_BUF[1 + 1] * 1000 + READ_BUF[1 + 2] * 100 + READ_BUF[1 + 3] * 10 + READ_BUF[1 + 4] * 1;  // convert to integer
    int a2 = READ_BUF[6 + 0] * 10000 + READ_BUF[6 + 1] * 1000 + READ_BUF[6 + 2] * 100 + READ_BUF[6 + 3] * 10 + READ_BUF[6 + 4] * 1;
    angle_output[0] = a1 / 100.0 - 360.0;  // do the math
    angle_output[1] = a2 / 100.0 - 360.0;
}

void get_soft_hard(bool s1, bool s2) {
    basic_message_get_debug(CMD_GET_SOFT_HARD);
}

void set_soft_hard(bool s1, bool s2) {
    setup_write_buffer_for_input(CMD_SET_SOFT_HARD);

    WRITE_BUF[5] = s1;
    WRITE_BUF[10] = s2;

    send(CMD_SET_SOFT_HARD);
}

// TODO make it round correctly instead of roudning down (+0.5)
void set_angles(double* angle_input) {
    // angleToSend = IntToString(360 * divisor + (desiredAngle * divisor))
    std::string s1 = ZeroPadNumber2Str((int)(360 * 10 + (angle_input[0] * 10)),4);  // do the math and convert to string
    std::string s2 = ZeroPadNumber2Str((int)(360 * 10 + (angle_input[1] * 10)),4);

    setup_write_buffer_for_input(CMD_SET_ANGLES);

    WRITE_BUF[1 + 0] = s1[0];
    WRITE_BUF[1 + 1] = s1[1];
    WRITE_BUF[1 + 2] = s1[2];
    WRITE_BUF[1 + 3] = s1[3];  // convert to chars for the write buffer
    WRITE_BUF[6 + 0] = s2[0];
    WRITE_BUF[6 + 1] = s2[1];
    WRITE_BUF[6 + 2] = s2[2];
    WRITE_BUF[6 + 3] = s2[3];

    send_recv(CMD_SET_ANGLES, NOMINAL_RETURN_MSG_BYTES);
}

// TODO make it round correctly instead of roudning down (+0.05)
void set_angles_100(double* angle_input) {
    // angleToSend = IntToString(360 * divisor + (desiredAngle * divisor))
    std::string s1 = ZeroPadNumber2Str((int)(360 * 100 + (angle_input[0] * 100)),5);  // do the math and convert to string
    std::string s2 = ZeroPadNumber2Str((int)(360 * 100 + (angle_input[1] * 100)),5);

    setup_write_buffer_for_input(CMD_SET_ANGLES_100);

    WRITE_BUF[1 + 0] = s1[0];
    WRITE_BUF[1 + 1] = s1[1];
    WRITE_BUF[1 + 2] = s1[2];
    WRITE_BUF[1 + 3] = s1[3];  // convert to chars for the write buffer
    WRITE_BUF[1 + 4] = s1[4];
    WRITE_BUF[6 + 0] = s2[0];
    WRITE_BUF[6 + 1] = s2[1];
    WRITE_BUF[6 + 2] = s2[2];
    WRITE_BUF[6 + 3] = s2[3];
    WRITE_BUF[6 + 4] = s2[4];

    print_write_buf();

    send_recv(CMD_SET_ANGLES_100, NOMINAL_RETURN_MSG_BYTES);
}


void stop_rotor() {
    send_recv(CMD_STOP, NOMINAL_RETURN_MSG_BYTES);
}

void response_time_stats() {
}

void set_motor_direction(int RL, int UD) {
    // sets the direction of the motor in which to apply the power setting from CMD_POWER

    // mmCmdStop = 0x00

    // mmCmdLeft = 0x01
    // mmCmdRight = 0x02
    // mmCmdUp = 0x04
    // mmCmdDown = 0x08
    // mmCmdLeftUp = 0x05
    // mmCmdRightUp = 0x06
    // mmCmdLeftDown = 0x09
    // mmCmdRightDown = 0x0A

    // half a byte indicating the desired direction
    // X  X  X  X
    // L  R  U  D
    // Either left or right and either up or down, alternatively all zero for stop

    bool L, R, U, D;
    R = RL > 0; L = RL < 0;
    U = UD > 0; D = UD < 0;

    setup_write_buffer_for_input(CMD_MOTORS);

    uint8_t cmd_byte = ((L << 0) + (R << 1) + (U << 2) + (D << 3));

    WRITE_BUF[1] = cmd_byte;

    send(CMD_MOTORS);
}

void set_motor_power(int p1, int p2) {
    setup_write_buffer_for_input(CMD_POWER);

    WRITE_BUF[5] = p1;
    WRITE_BUF[10] = p2;

    send_recv(CMD_POWER, NOMINAL_RETURN_MSG_BYTES);

    if (DEBUG_COMMS) {
        print_read_buf();
    }
}


void command_motors(int control_input[2], double* angle_output) {
    static int last_directions[2] = {0, 0};
    static int last_power[2] = {0, 0};
    angle_output[0] = NAN;
    angle_output[1] = NAN;

    if (std::abs(control_input[0]) < min_power && std::abs(control_input[1]) < min_power) {
        set_motor_direction(0,0); // stop
        return;
    }
    // minimize commands sendt to the rotor controller
    if (last_directions[0] != sign(control_input[0]) || last_directions[1] != sign(control_input[1])) { 
        set_motor_direction(sign(control_input[0]),sign(control_input[1]));
        last_directions[0] = sign(control_input[0]);
        last_directions[1] = sign(control_input[1]);
    }
    if (last_power[0] != abs(control_input[0]) || last_power[1] != abs(control_input[1])) {
        set_motor_power(abs(control_input[0]),abs(control_input[1]));
        last_power[0] = abs(control_input[0]);
        last_power[1] = abs(control_input[1]);
        decode_angles(angle_output);
    }
    else {
        get_angles_100(angle_output);
    }
}

void control_step(double reference[2], double value[2], int* output, double dt, double max_vel = 1.5, double max_acc = 25) {
    static double prev_error[2] = {0,0};
    static double prev_value[2] = {value[0],value[1]};
    static double v_dot_prev[2] = {0,0};
    double Kp = 0.2 * max_vel;
    double Kd = 0.0000 * max_vel;
    Kd = 0;

    for (int i = 0; i < 2; i++) {
        double error = reference[i] - value[i];
        double error_dot = 0;
        if (dt != 0) {
            error_dot = (error - prev_error[i]) / dt;
        }

        double u = Kp * error + Kd * error_dot;

        double v_dot = 0;
        double desired_acc = 0;
        double acc = 0;
        if (dt != 0) { // This should be true for all but the first step
            v_dot = (value[i] - prev_value[i]) / dt; // possibly quite noisy;
            v_dot = exp_smoothing(v_dot, v_dot_prev[i]); // so filter it
            v_dot_prev[i] = v_dot;

            desired_acc = (u - v_dot) / dt;
            acc = std::max(std::min(desired_acc, max_acc), -max_acc); // saturate the acceleration for safety purposes

            u = v_dot + acc * dt;
        }
        // convert from degrees per second to percentage, [max_vel] deg/s = 100 %
        output[i] = (int) (u * max_power / max_vel);
        int sgn = sign(output[i]);
        if (std::abs(u) < 0.01) {
            output[i] = 0;
        } else {
            output[i] = sgn * std::max(std::min(std::abs(output[i]), max_power), min_power); // saturate the commanded velocity
        }
        prev_error[i] = error;
        prev_value[i] = value[i];
        // std::cout << reference[i] << " " << value[i] << " " << error << " " << error_dot << " " << v_dot << " " << desired_acc << " " << acc << " " << u <<  " " << output[i] << std::endl;
    }
    // std::cout << dt << std::endl;
}

void track_trajectory(std::string trajectory_file_path) {
    // Setup
    // get trajectory from file
    // Check feasibility of trajectory
    //   * Time window
    //   * Angular range
    //   * Angular velocity
    // print info about the trajectory for validation
    
    // Run 
    // wait for confirmation
    //
    // go to starting position
    
    std::cout << trajectory_file_path << std::endl;
    std::fstream fs;
    fs.open(trajectory_file_path,std::ios::in);

    if (!fs.good()) {
        std::cout << "File could not be read" << std::endl;
        return;
    }
    
    std::vector<std::array<double,3>> traj;
    std::string header;
    std::getline(fs, header);
    
    std::string line;
    while (std::getline(fs,line)) {
        // std::cout << line << std::endl;
        std::string t_s,az_s,el_s;
        double t,az,el;
        std::stringstream line_s(line);
        std::getline(line_s, t_s, ','); std::getline(line_s, az_s, ','); std::getline(line_s, el_s, ',');
        t = std::stod(t_s); az = std::stod(az_s); el = std::stod(el_s);
        std::array<double,3> tmp = {t,az,el};
        traj.push_back(tmp);
    }

    std::cout << traj.size() << std::endl;


    auto now_micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    double now_s = now_micro_sec/MICRO_SEC_PER_SEC;

    double end_time = traj[traj.size()-1][0];
    double start_time = traj[0][0];

    if (now_s > traj[0][0]) {
        if (now_s < traj[traj.size()-1][0]) {
            std::cout << "Pass is underway, there are " << (int)now_s << " seconds left, track anyway? [y/N]: ";
            std::string ans;
            getline(std::cin, ans);
            if (! (ans == "y" || ans == "Y")) {
                return;
            }
        } else {
            int end_duration = (int)(traj[traj.size()-1][0] - now_s);
            std::cout << "The pass has ended " << end_duration/60 << " minutes and " << end_duration - (end_duration/60)*60  << " seconds ago, exiting." << std::endl;
            return;
        }
    } else {
        int sec_until = traj[0][0] - (int)now_s;
        std::cout << "The pass will happen in " << sec_until/60 << " minutes and " << sec_until - (sec_until/60)*60  << " seconds, wait for pass and track? [Y/n]: ";
        std::string ans;
        getline(std::cin, ans);
        if (ans == "n" || ans == "N") {
            return;
        }
    }

    // TODO do workspace boundary check

    // TODO do max axis speed check

    // setup connection
    // if (setup_USB_UART_connection() != 0) return;
    // Goto start position
    double start_angle[2] = {traj[0][1], traj[0][2]};
    printf("Going to start position. Az: %.2f El: %.2f\n",traj[0][1], traj[0][2]);
    // set_angles_100(start_angle);

    // wait
    if (now_s + 10 < start_time) {
        std::cout << "Waiting for start of pass" << std::endl;
        while (now_s + 10 < start_time) {
            now_micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            now_s = now_micro_sec/MICRO_SEC_PER_SEC;
            sleep(1);
        }
        printf("10 seconds to pass");
    }

    // track trajectory

    double angles_ref[2] = {0,0};

    double angles_measured[2];
    int control_signal[2];

    // get_angles_100(angles_measured);
    angles_ref[0] = start_angle[0]; angles_ref[1] = start_angle[1];

    int traj_index = 0;

    now_micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    now_s = now_micro_sec/MICRO_SEC_PER_SEC;
    double prev_time_s = now_s;
    while (now_s < end_time) {

        now_micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        now_s = now_micro_sec/MICRO_SEC_PER_SEC;
        double dt = (prev_time_s - now_s);
        prev_time_s = now_s;

        if (now_s > traj[traj_index][0]) {
            traj_index++;
        }

        // lerp between the points in the trajectory
        double traj_angles[2] = {traj[traj_index][1], traj[traj_index][2]};
        double traj_angles_next[2] = {traj[traj_index+1][1], traj[traj_index+1][2]};
        double t = (now_s - traj[traj_index][0]) / (traj[traj_index+1][0] - traj[traj_index][0]);
        lerp_arr(traj_angles,traj_angles_next,t,angles_ref,2);

        control_step(angles_ref, angles_measured, control_signal, dt, 6.0, 100.0);
        if (angular_distance(angles_ref, angles_measured) < 0.05) {
            int zero[2] = {0,0};
            // command_motors(zero, angles_measured);
        } else {
            // command_motors(control_signal, angles_measured);
        }
        if (true || std::isnan(angles_measured[0]) || std::isnan(angles_measured[1]) || angular_distance(angles_ref, angles_measured) < 0.5) {
            // get_angles_100(angles_measured);
        }

        usleep((int)std::max(0.0, (0.1 - dt) * MICRO_SEC_PER_SEC));
    }
}