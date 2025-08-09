#include "rotor_control.cpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <math.h>

void do_circle_test_power(double az_startpoint, double el_startpoint, int power, double period = 5, double run_time = 10.0) {
    double angles_startpoint[2];
    angles_startpoint[0] = az_startpoint;
    angles_startpoint[1] = el_startpoint;

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", timeinfo);
    std::string str(buffer);

    std::ofstream data_file;

    data_file.open((std::string) "test_data/circle_" + buffer + "_T_" + std::to_string((int)period) + "_Pow_" + std::to_string(power) + "_" + std::to_string((int)az_startpoint) + "_" + std::to_string((int)el_startpoint) + ".csv");
    data_file << "t,p_az,p_el,a_az,a_el" << std::endl;
    
    set_motor_power(80, 80);        // set initial power to go to starting position (limit wobble of the dish assembly)
    set_angles(angles_startpoint);  // goto staring position
    
    double angles[2];
    get_angles(angles);
    printf("Moving to setpoint %.2f, %.2f ...\n", angles_startpoint[0], angles_startpoint[1]);
    // wait for it to be done moving
    while (abs(angles[0] - angles_startpoint[0]) > 0.2 || abs(angles[1] - angles_startpoint[1]) > 0.2) {
        sleep(5);
        get_angles(angles);
        printf("Current position: %.2f,%.2f\n", angles[0], angles[1]);
    }
    
    
    
    printf("Waiting for 4 second to settle after moving\n");
    sleep(4);  // wait for the rotor to settle
    printf("Running circle test. Starting-point: Az %.2f El %.2f, period: %.2f [m], Power: %d [%%]\n", az_startpoint, el_startpoint, period, power);
    
    // begin collecting data
    get_angles_100(angles);
    std::chrono::time_point<std::chrono::system_clock> t_start = std::chrono::system_clock::now();
    // save current angle and commanded power (0)
    double t = 0.0;
    // data_file << 0 << "," << 0 << "," << 0 << "," << angles[0] << "," << angles[1] << std::endl;
    
    // command rotor and collect data points as fast as possible
    double angle_output[2];
    double motor_commands[2] = {0, 0};
    while (t < run_time) {
        get_angles_100(angles);
        int control_input[2] = {(int)(power*cos(t * 2 * M_PI / period)), (int)(power*sin(t * 2 * M_PI / period))};
        command_motors(control_input,angle_output);
        
        // Time is taken after angles are retrieved and motors are commanded from the rotor (2 x 16 ms) , may lead to inaccurate delays
        t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t_start).count();
        t /= 1000000.;
        
        data_file << t << "," << control_input[0] << "," << control_input[1] << "," << angles[0] << "," << angles[1] << std::endl;
        
    }

    set_motor_direction(0, 0);  // stop the rotor after <run_time> seconds
    data_file.close();
}


void do_step_response(double az_startpoint, double el_startpoint, int RL_inp, int UD_inp, double step_time = 3) {
    double angles_startpoint[2];
    angles_startpoint[0] = az_startpoint;
    angles_startpoint[1] = el_startpoint;
    // power in azimuth and elevation

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", timeinfo);
    std::string str(buffer);

    std::ofstream data_file;
    data_file.open((std::string) "test_data/step_" + buffer + "_az_" + std::to_string(RL_inp) + "_el_" + std::to_string(UD_inp) + "_" + std::to_string((int)az_startpoint) + "_" + std::to_string((int)el_startpoint) + ".csv");
    data_file << "t,p_az,p_el,a_az,a_el" << std::endl;

    set_motor_power(80, 80);      // set initial power to go to starting position
    set_angles(angles_startpoint);  // goto staring position

    double angles[2];
    get_angles(angles);
    printf("Moving to setpoint %.2f, %.2f ...\n", angles_startpoint[0], angles_startpoint[1]);
    // wait for it to be done moving
    while (abs(angles[0] - angles_startpoint[0]) > 0.2 || abs(angles[1] - angles_startpoint[1]) > 0.2) {
        sleep(5);
        get_angles(angles);
        printf("Current position: %.2f,%.2f\n", angles[0], angles[1]);
    }

    set_motor_power(abs(RL_inp), abs(UD_inp));  // set power to testing value (100%)

    printf("Waiting for 4 second to settle after moving\n");
    sleep(4);  // wait for the rotor to settle
    printf("Running step response. Set-point: Az %.2f El %.2f, Power: Az %d El %d\n", az_startpoint, el_startpoint, RL_inp, UD_inp);

    // begin collecting data
    get_angles_100(angles);
    std::chrono::time_point<std::chrono::system_clock> t_start = std::chrono::system_clock::now();
    // save current angle and commanded power
    double t = 0.0;
    data_file << 0 << "," << 0 << "," << 0 << "," << angles[0] << "," << angles[1] << std::endl;

    set_motor_direction(sign(RL_inp), sign(UD_inp));
    // collect data points as fast as possible
    while (t < step_time) {
        get_angles_100(angles);
        // Time is taken after angles are retrieved from the rotor, may lead to inaccurate delays
        t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t_start).count();
        t /= 1000000;

        data_file << t << "," << RL_inp << "," << UD_inp << "," << angles[0] << "," << angles[1] << std::endl;

        // rotor commands are blocking so no need to sleep due to "get_angles_100"
        // sleep(0.016);
    }

    set_motor_direction(0, 0);  // stop the rotor after <step_time> seconds
    data_file.close();
}


int main(int argc, char *argv[]) {
    if (setup_USB_UART_connection() != 0) return -1;

    //               Az, El, RL_inp, UD_inp
    // do_step_response( 0, 90,      0,    100);    // Pointing up      , moving backwards (positive elevation)
    // do_step_response( 0, 90,      0,   -100);    // Pointing up      , moving forwards (negative elevation)
    // do_step_response( 0, 90,   -100,      0);    // Pointing up      , moving left/clockwise(from above, negative azimuth)
    // do_step_response( 0, 90,    100,      0);    // Pointing up      , moving right/counter-clockwise(from above, positive azimuth)
    
    // // elevation 5 degrees above the horizon to not meet the motor endstop
    // do_step_response( 0,  5,      0,    100);    // Pointing forwards, moving up (positive elevation)
    // // do_step_response( 0,  5,      0,   50-100)     Cannot move downwards. Do not run this
    // do_step_response( 0,  5,   -100,      0);    // Pointing forwards, moving left/clockwise(from above, negative azimuth)
    // do_step_response( 0,  5,    100,      0);    // Pointing forwards, moving right/counter-clockwise(from above, positive azimuth)

    do_circle_test_power(0, 70, 80, 8, 30);

    return 0;
}
