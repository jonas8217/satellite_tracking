//
// Created by Admin on 16.06.2021.
//

#ifndef SPID_MD0X_SETTINGS_PARAMS_LEGACY_H
#define SPID_MD0X_SETTINGS_PARAMS_LEGACY_H

#include <stdint.h>

typedef enum STATE {
    OFF = 0, ON = 1
} STATE;
/**
 * Motor kind
 */
typedef enum MOTOR_KIND : uint8_t {
    KIND_AZIMUTH = 0, /**< Azimuth */
    KIND_ELEVATION = 1, /**< Elevation */
    KIND_X, /**< X */
    KIND_Y, /**< Y */
    KIND_AZIMUTH_AS_X, /**< X, but received as Azimuth */
    KIND_ELEVATION_A_Y, /**< Y, but received as Elevation */
} MOTOR_KIND;

typedef enum MOTOR_TYPE : uint8_t {
    TYPE_DIGITAL = 0,
    TYPE_ANALOG = 1
} MOTOR_TYPE;

typedef enum MOTOR_INPUT : uint8_t {
    INPUT_MECHANIC = 0,
    INPUT_ELECTRONIC = 1,
    INPUT_CAN = 2,
    INPUT_UNKNOWN = 0xff,
} MOTOR_INPUT;

typedef enum : uint8_t {
    INC_MOTOR_POWER = 0,
    DEC_MOTOR_POWER = 1
} MOTOR_POWER;

typedef enum seMotorTemplate : uint8_t {
    mtNone = 0,
    mtAz,
    mtEl,
    mtAzEl,
    mtAzAz,
    mtElEl,
    mtXY,
    mtAzElXY,  // receive az, el, translate to XY
} seMotorTemplate;

typedef enum seMotorShowAzimuth : uint8_t {
    saNormal,
    saMath,
} seMotorShowAzimuth;

typedef enum seMotorSoftStart : uint8_t {
    sstHard,
    sstSoft,
} seMotorSoftStart;

typedef enum seMotorControlPort : uint8_t {
    cptNONE,
    cptCOM0,
    cptCOM1,
    cptETH,
    cptUSB,
} seMotorControlPort;

typedef enum seMotorControlProtocol : uint8_t {
    cprtNONE,
    cprtROT1,
    cprtROT2,
    cprtYaesu,
    cprtMD01,
    cprtHyGain,
    cprtBrite,
} seMotorControlProtocol;

typedef enum seMotorEncoderGear {
    megCustom = 0,
    meg1_0 =      1000000,
    meg0_5 =       500000,
    meg0_035156 =   35156,
    meg0_031250 =   31250,
    meg0_023438 =   23438,
    meg0_013560 =   13560,
} seMotorEncoderGear;

typedef enum seMotorCurrentMVPerA : uint16_t {
    mcmpaCustom = 0,
    mcmpa5A = 185,
    mcmpa20A = 100,
    mcmpa30A = 66,
} seMotorCurrentMVPerA;


typedef enum seComBaudRate : uint8_t {
    cbr600 = 0,
    cbr1200,
    cbr2400,
    cbr4800,
    cbr9600,
    cbr19200,
    cbr38400,
    cbr57600,
    cbr115200,
    cbr230400,
    cbr460800,
} seComBaudRate;

typedef enum seDisplayResolution : uint8_t {
    dr1Degree,
    dr0_5Degree,
    dr0_1Degree,
    dr0_05Degree,
    dr0_01Degree,
} seDisplayResolution;

typedef enum seDataVersion : uint32_t {
    dvLegacy = 0xfffffff9,
    dvNewV2_0 = 0xfffffff8,
    dvNewV2_0_1 = 0xfffffff7,
} seDataVersion;

typedef struct  {
    float       min_angle;
    float       max_angle;
    float       gear;            /* degrees per pulse */
    //float       stop_at;         /* UNUSED: wyprzedzenie przed miêkki stop */
    uint8_t     min_power;       /* percent: 0%-100% */
    uint16_t    canEncoder;      /* enccanSupported */
    STATE       state;        /* ON, OFF */
    MOTOR_TYPE  type;
    MOTOR_KIND  kind;            /* AZIMUTH, ELEVATION */
    MOTOR_INPUT input;
    uint8_t     start_time[3];   /* seconds: 0s-30s */
    uint8_t     start_power[3];  /* percent: 0%-100% */
    uint8_t     stop_time[3];    /* seconds: 0s-30s */
    uint8_t     stop_power[3];   /* percent: 0%-100% */
    uint8_t     max_power;       /* percent: 0%-100% */
    uint8_t     pulse_timeout;    /* 1s - 10s co 1s */
} MOTOR_PARAMS;

typedef struct  {
    uint32_t power;  // current speed (CS in block diagrams)
    uint32_t tpower; // target speed (TS in block diagrams)
    MOTOR_POWER flag;
    uint8_t step;
    uint8_t ppower;
    uint8_t usedIndex;
    uint8_t doStop;  // if 1 then start holding motor
} MOTOR_RAMP;

typedef struct  {
    int32_t pulses[2];
    uint8_t spin[2];
    uint8_t pins;
    uint8_t old_pins;
    uint8_t show_toPulses;
} MOTOR_VARS;


typedef struct  {
    uint8_t xxx_1; //control_by;
    uint8_t xxx_2; //assigned_to;
    uint8_t xxx_3; //protocol;
    seComBaudRate baud;
    uint8_t data_bits;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t not_used;
} COM_PARAMS;

typedef struct {
    STATE enabled;
    float maxCurrent; // in A
    uint16_t maxOCTimeMS; // overcurrent time in ms
    uint16_t mvPerAmper; // mv/A
    uint8_t reserved[16];
} CURRENT_LIMITING;

typedef struct  ROT_FRAM_PARAMS  {
// -------------------------------- DATAVER fffffffc
    seDataVersion data_ver;
    MOTOR_PARAMS motor[2];
    COM_PARAMS com[2];
    MOTOR_VARS motors;
    CURRENT_LIMITING currentLimiting[2];
    uint8_t unused[305];
    uint32_t motor_pins;
    seMotorSoftStart  manual_ctrl_start; // 0 - HARD; 1 - SOFT
    seMotorSoftStart  manual_ctrl_stop;  // 0 - HARD; 1 - SOFT
    seComBaudRate  usb_baudrate;
    STATE  use_short_way;
    STATE satellite_mode;
    seDisplayResolution  displayResolution;
    STATE    mouse_control;
// -------------------------------- DATAVER fffffffb
    STATE    usb_state;
    STATE    eth_state;
    seMotorTemplate  motor_template;
    seMotorControlPort  motor_control1;
    seMotorControlProtocol  motor_protocol1;
    seMotorControlPort  motor_control2;
    seMotorControlProtocol  motor_protocol2;
// -------------------------------- DATAVER fffffffa
    STATE    com_state[2];
// -------------------------------- DATAVER fffffff9
    seMotorShowAzimuth    az_show_math;
    STATE    pair_rotors;
// -------------------------------- DATAVER fffffff8
}	ROT_FRAM_PARAMS;

#endif //SPID_MD0X_SETTINGS_PARAMS_LEGACY_H
