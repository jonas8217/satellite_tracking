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

void eraseLines(int count) {
    if (count > 0) {
        printf("\x1b[2K"); // Delete current line
        // i=1 because we included the first line
        for (int i = 1; i < count; i++) {
            printf("\x1b[1A\x1b[2K");
        }
        printf("\r"); // Resume the cursor at beginning of line
    }
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

    uint8_t motor_1_power = 0; //az
    uint8_t motor_2_power = 0; //el

    uint8_t motor_directions = 0; // DURL

    std::mutex rw_mutex;
    std::thread sim_thread;
    bool unread_input = false;

    void sim_rotor_loop();
    void sim_rotor_step(double dt);
    void handle_write_input();
    void setup_read_buffer_for_output();

    void simple_controller();

    void get_motor_angles();
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

void simulated_rotor::setup_read_buffer_for_output() {
    for (int i = 0; i < 12; i++) {
        OUT_BUF[i] = 0;
    }
    OUT_BUF[0] = 0x57;
    OUT_BUF[11] = 0x20;
}

void simulated_rotor::simple_controller() {
    double az_err = az_target - az;
    double el_err = el_target - el;
    // static bool once = true;
    uint8_t L = az_err < 0;
    uint8_t R = az_err > 0;
    uint8_t U = el_err > 0;
    uint8_t D = el_err < 0;
    if (abs(az_err) > 0.05) {
        motor_1_power = 100;
    } else {
        motor_1_power = 0;
        L = R = 0;
    }
    if (abs(el_err) > 0.05) {
        motor_2_power = 100;
    } else {
        motor_2_power = 0;
        U = D = 0;
    }
    motor_directions = ((L << 0) + (R << 1) + (U << 2) + (D << 3));
    if (motor_directions == 0) {
        angle_target_set = false;
    }
    // if (once) {
    //     once = false;
    //     printf("%f %f %f %f %f %f\n", az, az_target, az_err, el, el_target, el_err);
    //     printf("%d %d %d\n", motor_1_power, motor_2_power, motor_directions);
    // }
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
    bool _unread_input = true;
    while (_unread_input) {
        {
            const std::lock_guard<std::mutex> lock(rw_mutex);
            _unread_input = unread_input;
        }
    }

    const std::lock_guard<std::mutex> lock(rw_mutex);
    if (__n > _TRANSMIT_MSG_LEN) {
        printf("[Simulated rotor] Cannot write to buffer, n: %lu greater than max message length: %d\n",__n,_TRANSMIT_MSG_LEN);
        return;
    }
    std::memcpy(INP_BUF,__buf, __n);
    unread_input = true;
}

void simulated_rotor::get_motor_angles()
{
     setup_read_buffer_for_output();

    std::string s1 = ZeroPadNumber2Str((int)(360 * 10 + (az * 10)),5);  // do the math and convert to string
    std::string s2 = ZeroPadNumber2Str((int)(360 * 10 + (el * 10)),5);

    OUT_BUF[1 + 0] = s1[0]-48;
    OUT_BUF[1 + 1] = s1[1]-48;
    OUT_BUF[1 + 2] = s1[2]-48;
    OUT_BUF[1 + 3] = s1[3]-48;
    OUT_BUF[1 + 4] = 0xa;
    OUT_BUF[6 + 0] = s2[0]-48;
    OUT_BUF[6 + 1] = s2[1]-48;
    OUT_BUF[6 + 2] = s2[2]-48;
    OUT_BUF[6 + 3] = s2[3]-48;
    OUT_BUF[6 + 4] = 0xa;
}

void simulated_rotor::get_motor_angles_100() {

    setup_read_buffer_for_output();

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
    int a1 = 0;
    int a2 = 0;
    for (int i = 0; i < 5; i++) {
        a1 += (INP_BUF[1+i] - 48) * pow(10,4-i);
        a2 += (INP_BUF[6+i] - 48) * pow(10,4-i);
    }
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
    get_motor_angles();
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
    int RL = ((motor_directions & 0b11) == 0) ? 0 : (((motor_directions & 0b11) == 1) ? -1 : 1);
    int DU = ((motor_directions >> 2) == 0) ? 0 : ((motor_directions >> 2 == 1) ? 1 : -1);
    // printf("%d\t%d\t%d\t%d\r", motor_directions, RL, DU,(motor_directions & 2));
    // integarate the current power (aka. speed percentage)
    az += az_speed * RL * (motor_1_power / 100.0) * dt;
    el += el_speed * DU * (motor_2_power / 100.0) * dt;
    // printf("%d\n%d\n%d\n%d\n%d\n%d\n%f", motor_directions, motor_1_power, motor_2_power, RL, DU, motor_directions, dt);
    // eraseLines(7);
}



void simulated_rotor::sim_rotor_loop()
{
    auto now_micro_sec = get_microseconds_now();
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
            }
        }
        if (angle_target_set) {
            simple_controller();
        }

        now_micro_sec = get_microseconds_now();
        double dt = now_micro_sec/_MICRO_SEC_PER_SEC - t;
        t = now_micro_sec/_MICRO_SEC_PER_SEC;
        sim_rotor_step(dt);
        usleep(1000);
    }
}

simulated_rotor::~simulated_rotor()
{
    if (run) {
        run = false;
        sim_thread.join();
    }
}


