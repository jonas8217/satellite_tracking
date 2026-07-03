#include <unistd.h>
#include <stdio.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <cstring>
#include <atomic>

#include "utils.cpp"

void print_buf(uint8_t* buf, int size) {
    for (int i = 0; i < size; i++) {
        printf("0x%02x ", buf[i]);
    }
    printf("\n");
}

class simulated_rotor
{
private:
    std::atomic_bool run = false;

    static constexpr double _MICRO_SEC_PER_SEC = 1000000.0;
    static constexpr int _MAX_RETURN_MSG_LEN = 43;
    static constexpr int _TRANSMIT_MSG_LEN = 13;

    enum _MSG_TYPE {
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

    static constexpr uint8_t _MSG_ARRAYS[][13] = {
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

    uint8_t OUT_BUF[_MAX_RETURN_MSG_LEN];
    uint8_t INP_BUF[_TRANSMIT_MSG_LEN];

    double az_speed = 1.5; // deg/s
    double el_speed = 1.5;

    double az = 0;
    double el = 90;

    double az_target = 0;
    double el_target = 0;
    bool angle_target_set = false;

    int motor_1_power = 50; //az
    int motor_2_power = 50; //el

    int motor_directions = 0; // DURL

    std::mutex rw_mutex;
    std::thread sim_thread;
    bool unread_input = false;

    void sim_rotor_loop();
    void sim_rotor_step(double dt);
    void handle_write_input();
    void setup_read_buffer_for_output(_MSG_TYPE type);

    void get_motor_angles_100();
    void set_angles_100();
    void set_directions();
    void set_power();
    void stop_rotor();

public:
    simulated_rotor();
    void setup();
    void write(uint8_t *__buf, size_t __n);
    int read(uint8_t *__buf, size_t __nbytes);
    bool running();
    ~simulated_rotor();
};

simulated_rotor::simulated_rotor()
{
}

void simulated_rotor::setup()
{
    sim_thread = std::thread(&simulated_rotor::sim_rotor_loop, this);
}

void simulated_rotor::setup_read_buffer_for_output(_MSG_TYPE type) {
    for (int i = 0; i < 12; i++) {
        OUT_BUF[i] = 0;
    }
    OUT_BUF[0] = 0x57;
    OUT_BUF[11] = 0x20;
}

int simulated_rotor::read(uint8_t *__buf, size_t __nbytes)
{
    bool _unread_input = true;
    while (_unread_input) {
        {
            const std::lock_guard<std::mutex> lock(rw_mutex);
            _unread_input = unread_input;
        }
    }

    const std::lock_guard<std::mutex> lock(rw_mutex);
    if (OUT_BUF[0] == 0) {
        printf("Nothing to read\n");
        return 0;
    }
    std::memcpy(__buf, OUT_BUF, __nbytes);
    // std::memset(OUT_BUF, 0, _MAX_RETURN_MSG_LEN); // not necessary if handled by the rotor controller correctly
    return __nbytes;
}

void simulated_rotor::write(uint8_t* __buf, size_t __n)
{
    const std::lock_guard<std::mutex> lock(rw_mutex);
    if (__n > _TRANSMIT_MSG_LEN) {
        printf("[Simulated rotor] Cannot write to buffer, n: %lu greater than max message length: %d\n",__n,_TRANSMIT_MSG_LEN);
        return;
    }
    std::memcpy(INP_BUF,__buf, __n);
    unread_input = true;
}

void simulated_rotor::get_motor_angles_100() {

    setup_read_buffer_for_output(CMD_GET_MOTOR_ANGLES_100);

    std::string s1 = ZeroPadNumber2Str((int)(360 * 100 + (az * 100)),5);  // do the math and convert to string
    std::string s2 = ZeroPadNumber2Str((int)(360 * 100 + (el * 100)),5);

    OUT_BUF[1 + 0] = s1[0]-48;
    OUT_BUF[1 + 1] = s1[1]-48;
    OUT_BUF[1 + 2] = s1[2]-48;
    OUT_BUF[1 + 3] = s1[3]-48;
    OUT_BUF[1 + 4] = s1[4]-48;
    OUT_BUF[6 + 0] = s2[0]-48;
    OUT_BUF[6 + 1] = s2[1]-48;
    OUT_BUF[6 + 2] = s2[2]-48;
    OUT_BUF[6 + 3] = s2[3]-48;
    OUT_BUF[6 + 4] = s2[4]-48;
}

void simulated_rotor::set_angles_100() {
    int a1 = INP_BUF[1 + 0] * 10000 + INP_BUF[1 + 1] * 1000 + INP_BUF[1 + 2] * 100 + INP_BUF[1 + 3] * 10 + INP_BUF[1 + 4] * 1;  // convert to integer
    int a2 = INP_BUF[6 + 0] * 10000 + INP_BUF[6 + 1] * 1000 + INP_BUF[6 + 2] * 100 + INP_BUF[6 + 3] * 10 + INP_BUF[6 + 4] * 1;
    az_target = a1 / 100.0 - 360.0;
    el_target = a2 / 100.0 - 360.0;
    angle_target_set = true;
    get_motor_angles_100();
}

void simulated_rotor::set_directions() {
    motor_directions = INP_BUF[1];
    angle_target_set = false;
}

void simulated_rotor::set_power() {
    motor_1_power = INP_BUF[5];
    motor_2_power = INP_BUF[10];
    angle_target_set = false;
}

void simulated_rotor::stop_rotor() {
    motor_1_power = 0;
    motor_2_power = 0;
    motor_directions = 0;
    angle_target_set = false;
}

void simulated_rotor::handle_write_input() {
    // implement CMD_GET_MOTOR_ANGLES_100, CMD_SET_ANGLES_100, CMD_MOTORS, CMD_POWER
    switch (INP_BUF[11])
    {
    case 0:
        // No new command
        break;
    case _MSG_ARRAYS[CMD_GET_MOTOR_ANGLES_100][11]:
        get_motor_angles_100();
        break;
    case _MSG_ARRAYS[CMD_SET_ANGLES_100][11]:
        set_angles_100();
        break;
    case _MSG_ARRAYS[CMD_MOTORS][11]:
        set_directions();
        break;
    case _MSG_ARRAYS[CMD_POWER][11]:
        set_power();
        break;
    case _MSG_ARRAYS[CMD_STOP][11]:
        stop_rotor();
        break;
    default:
        printf("[Simulated rotor] command not implemented '0x%02x'\n", INP_BUF[11]);
        break;
    }
    memset(INP_BUF, 0, _TRANSMIT_MSG_LEN); // clear buffer to indicate no message
}

bool simulated_rotor::running(){
    return run;
}

void simulated_rotor::sim_rotor_step(double dt)
{
    // get directions
    int RL = (motor_directions & 2 == 0) ? 0 : ((motor_directions & 2 == 1) ? 1 : -1);
    int DU = (motor_directions >> 2 == 0) ? 0 : ((motor_directions >> 2 == 1) ? 1 : -1);
    // integarate the current power (aka. speed percentage)
    az += az_speed * RL * (motor_1_power / 100) * dt;
    el += el_speed * DU * (motor_2_power / 100) * dt;
}

void simulated_rotor::sim_rotor_loop()
{
    auto now_micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    double t = now_micro_sec/_MICRO_SEC_PER_SEC;
    bool _angle_target_set = angle_target_set;
    printf("[Simulated rotor] Starting sim loop\n");
    run = true;
    while (run) {
        // handle input
        {
            const std::lock_guard<std::mutex> lock(rw_mutex);
            if (unread_input) {
                handle_write_input();
                unread_input = false;
                _angle_target_set = angle_target_set;
            }
        }
        now_micro_sec = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        double dt = now_micro_sec/_MICRO_SEC_PER_SEC - t;
        sim_rotor_step(dt);
    }
}

simulated_rotor::~simulated_rotor()
{
    if (run) {
        run = false;
        sim_thread.join();
    }
}


