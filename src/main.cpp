#include "rotor_control.cpp"

#include <thread>
#include <mutex>

void print_help() {
    printf("Commands:\n");
    printf("  restart\n");
    printf("  read\n");
    printf("  stop\n");
    printf("  set-angles [az/X] [el/Y]\n");
    printf("  set-power  [az/X] [el/Y]\n");
    // set-direction
    // get-config
}

#define MICRO_SEC_PER_SEC 1000000

volatile double angles_reference_input[2] = {0,0};
volatile bool stop = false;
std::mutex m;

void get_reference_input() {
    double angles[2] = {0,0};
    while (true) {
        std::string inp_buff;
        std::getline(std::cin, inp_buff);
        if (inp_buff.find("stop") == 0) {
            stop = true;
            return;
        }
        unsigned idx = inp_buff.find(" ");
        std::string X = inp_buff.substr(0, idx);
        std::string Y = inp_buff.substr(idx+1, inp_buff.length()-(idx+1));
        try {
            angles[0] = std::stod(X);
            angles[1] = std::stod(Y);
            {
                const std::lock_guard<std::mutex> lock(m);
                angles_reference_input[0] = angles[0];
                angles_reference_input[1] = angles[1];
            }
        } catch (std::invalid_argument& e) {
            std::cout << "Incorrect angle input format. Format as \"X Y\" (decimals allowed)." << std::endl;
        }
    }
}

double angular_distance(double a[2], double b[2]) {
    // approximate with Euclidean distance (correct for small angles)
    return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]));
}

int main(int argc, char *argv[]) {
    bool do_control = false;
    if (argc > 1) {  // test for commands
        if (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "help") == 0) {
            print_help();
            return 0;
        } else if (std::strcmp(argv[1], "track") == 0) {
            track_trajectory(argv[2]);
            return 0;
        } else if (std::strcmp(argv[1], "do-control") == 0) {
            do_control = true;
        } else {
            if (setup_USB_UART_connection() != 0) return -1;
        }

        if (std::strcmp(argv[1], "restart") == 0) {
            basic_message_get_debug(CMD_RESTART_DEVICE);
            printf("Restart sent successfully\n");

        } else if (std::strcmp(argv[1], "read") == 0) {
            double angle_output[2];
            get_angles_100(angle_output);
            if (argc == 2) {
                printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angle_output[0], angle_output[1]);
            } else {  // if another argument is given after "read" (any argument) print a stripped down version of the angles for good interfacing with other programs
                printf("%.2f,%.2f", angle_output[0], angle_output[1]);
            }

        } else if (std::strcmp(argv[1], "stop") == 0) {
            basic_message_get_debug(CMD_STOP);
            printf("Stop sent successfully\n");

        } else if (std::strcmp(argv[1], "set-angles") == 0) {
            double angles_inp[2];
            angles_inp[0] = std::stod(argv[2]);
            angles_inp[1] = std::stod(argv[3]);
            int m1_power = get_configuration(23).int_;
            int m2_power = get_configuration(46).int_;

            if (m1_power < 20 || m2_power < 20) {
                set_motor_power(80,80);
                printf("power set to 80 80, was %d %d\n", m1_power, m2_power);
            }

            set_angles_100(angles_inp);
            if (argc == 5 && std::strcmp(argv[4], "q") == 0) {
                printf("Angle %.2f %.2f set successfully\n", angles_inp[0], angles_inp[1]);
            } else {
                printf("Command sent successfully: %.2f, %.2f\n", angles_inp[0], angles_inp[1]);

                double angles_read[2];
                get_angles(angles_read);

                if (angular_distance(angles_inp, angles_read) < 0.15) {
                    printf("Done\n");

                } else {
                    double angles_read_prev[2];
                    angles_read_prev[0] = angles_read[0]; angles_read_prev[1] = angles_read[1];
                    printf("Waiting...\n");
                    usleep(1 * MICRO_SEC_PER_SEC);
                    while (true) {
                        get_angles_100(angles_read);
                        if (angular_distance(angles_inp, angles_read) < 0.1) {
                            printf("Done:\n");
                            printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angles_read[0], angles_read[1]);
                            break;
                        }
                        if (angles_read[0] == angles_read_prev[0] && angles_read[1] == angles_read_prev[1]) {
                            printf("Is motor stalled?\nConsider chaking the power settings on the rotor.\n");
                            break;
                        }
                        angles_read_prev[0] = angles_read[0]; angles_read_prev[1] = angles_read[1];
                        usleep(0.1 * MICRO_SEC_PER_SEC);
                    }
                }
            }

        } else if (std::strcmp(argv[1], "set-power") == 0) {
            int p1 = std::stoi(argv[2]);
            int p2 = std::stoi(argv[3]);
            set_motor_power(p1, p2);
            printf("Power set successfully\n");
        }
        // This command has the possibilty of running the rotor against one of the motors internal endstops, this may lock up one of the two motors or possibly damage the motors
        else if (std::strcmp(argv[1], "set-direction") == 0) {
            if (argc != 4) {
                printf("Movement duration not provided, direction not set\n");
            }
            int d = std::stoi(argv[2]);             // movement direction Right:0 Left:1 Up:2 Down:3
            double t = std::stod(argv[3]);          // time on seconds (decimal allowed)

            int RL = abs(1-d*2) == 1 ? 1-d*2 : 0;
            int UD = abs(1-(d-2)*2) == 1 ? 1-(d-2)*2 : 0;
            set_motor_direction(RL, UD);  // only one direction per message
            printf("Direction set successfully, running for %.3f\n", t);
            usleep((int)t * MICRO_SEC_PER_SEC);         // Microsecond sleep
            set_motor_direction(0, 0);  // Stop the motors
            printf("Rotor stopped\n");

        } else if (std::strcmp(argv[1], "get-config") == 0) {
            if (argc != 3) {
                printf("missing config field value\n");
            }
            int field_id = std::stoi(argv[2]);
            get_configuration(field_id);
            print_cfg();
        } else if ((std::strcmp(argv[1], "-h") != 0 && std::strcmp(argv[1], "help") != 0 && !do_control)) {
            print_help();
        }
    }

    if (DO_CONTROL || do_control) {
        if (setup_USB_UART_connection() != 0) return -1;

        double angles_ref[2] = {0,0};
        double angles_ref_old[2] = {0,0};

        double angles_measured[2];
        int control_signal[2];

        get_angles_100(angles_measured);
        angles_reference_input[0] = angles_measured[0]; angles_reference_input[1] = angles_measured[1];
        angles_ref[0] = angles_reference_input[1]; angles_ref[1] = angles_reference_input[1];

        std::thread thread(get_reference_input);
        auto start_time = std::chrono::high_resolution_clock::now();

        while (!stop) {
            { // scope for the mutex lock so it releases the lock when it exits the scope
                const std::lock_guard<std::mutex> lock(m);
                angles_ref[0] = angles_reference_input[0];
                angles_ref[1] = angles_reference_input[1];
            }
            // std::cout << angles_ref[0] << " " << angles_ref[1] << std::endl;
            // if (angles_ref_old[0] != angles_ref[0] || angles_ref_old[1] != angles_ref[1]) {
            //     angles_ref_old[0] = angles_ref[0];
            //     angles_ref_old[1] = angles_ref[1];

            //     // Simple
            //     // set_angles(angles_ref);
            //     printf("Angle %.2f %.2f set successfully\n", angles_ref[0], angles_ref[1]);
            // }

            // velocity based control loop
            auto end_time = std::chrono::high_resolution_clock::now();
            double dt = (end_time - start_time)/std::chrono::microseconds(1); // convert duration to microseconds
            dt = dt/MICRO_SEC_PER_SEC; // convert from microseconds to seconds
            start_time = end_time;

            control_step(angles_ref, angles_measured, control_signal, dt, 6.0, 100.0);
            if (angular_distance(angles_ref, angles_measured) < 0.05) {
                int zero[2] = {0,0};
                command_motors(zero, angles_measured);
            } else {
                command_motors(control_signal, angles_measured);
            }
            if (true || std::isnan(angles_measured[0]) || std::isnan(angles_measured[1]) || angular_distance(angles_ref, angles_measured) < 0.5) {
                get_angles_100(angles_measured);
            }

            usleep(std::max(0, (int) (0.1 - dt) * MICRO_SEC_PER_SEC));
        }
        thread.join();

    } else if (argc == 1) {  // Testing area
        // print_help();
        if (setup_USB_UART_connection() != 0) return -1;
        double angle_output[2];

        while (true) {
            unsigned long start = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
            get_angles_100(angle_output);
            printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angle_output[0], angle_output[1]);
            unsigned long now = std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
            printf("ms duration: %lu \n", now - start);
        }

        // double angle_output[2] = {0,0};
        // get_angles(angle_output);
        // printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angle_output[0], angle_output[1]);

        // get_angles_100(angle_output);
        // printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angle_output[0], angle_output[1]);

        // angle_output[0] = -24.5; angle_output[1] = 0.3;
        // set_angles(angle_output);

        // set_motor_direction(0,0,1,0);

        // printf("\nWrite buffer: ");
        // for (int i = 0; i < TRANSMIT_MSG_LEN; i++) {
        //     printf("0x%02x ",WRITE_BUF[i]);
        // }
        // printf("\n");

        // basic_message_get_debug(CMD_GET_SOFT_HARD);
        // set_soft_hard(0,0);
        // sleep(0.1);
        // basic_message_get_debug(CMD_GET_SOFT_HARD);

        // get_angles_100(angle_output);
        // printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angle_output[0], angle_output[1]);
        // set_motor_direction(0,0,1,0);
        // set_motor_power(0,40);

        // int t = 8;
        // printf("sleeping for %i seconds\n", t);
        // sleep(t);

        // get_angles_100(angle_output);
        // printf("Motor 1 (az/X) angle: %.2f\nMotor 2 (el/Y) angle: %.2f\n", angle_output[0], angle_output[1]);

        // printf("\nStopping rotor\n");
        // stop_rotor();
    }

    return 0;
}
