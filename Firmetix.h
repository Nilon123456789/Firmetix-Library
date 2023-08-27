#ifndef _FIRMETIX_H_
#define _FIRMETIX_H_

class Firmetix {

public:

    // firmware version - update this when bumping the version
    #define FIRMWARE_MAJOR 7
    #define FIRMWARE_MINOR 1
    #define FIRMWARE_PATCH 2

    #ifndef LED_BUILTIN
    #define LED_BUILTIN 2
    #endif

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    /*         Client Command Related Defines and Support               */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    // Commands Sent By The Client

    // Add commands retaining the sequential numbering.
    // The order of commands here must be maintained in the command_table.
    #define SERIAL_LOOP_BACK 0
    #define SET_PIN_MODE 1
    #define DIGITAL_WRITE 2
    #define ANALOG_WRITE 3
    #define MODIFY_REPORTING 4 // mode(all, analog, or digital), pin, enable or disable
    #define GET_FIRMWARE_VERSION 5
    #define ARE_U_THERE 6
    #define SERVO_ATTACH 7
    #define SERVO_WRITE 8
    #define SERVO_DETACH 9
    #define I2C_BEGIN 10
    #define I2C_READ 11
    #define I2C_WRITE 12
    #define SONAR_NEW 13
    #define DHT_NEW 14
    #define STOP_ALL_REPORTS 15
    #define SET_ANALOG_SCANNING_INTERVAL 16
    #define ENABLE_ALL_REPORTS 17
    #define RESET 18
    #define SPI_INIT 19
    #define SPI_WRITE_BLOCKING 20
    #define SPI_READ_BLOCKING 21
    #define SPI_SET_FORMAT 22
    #define SPI_CS_CONTROL 23
    #define ONE_WIRE_INIT 24
    #define ONE_WIRE_RESET 25
    #define ONE_WIRE_SELECT 26
    #define ONE_WIRE_SKIP 27
    #define ONE_WIRE_WRITE 28
    #define ONE_WIRE_READ 29
    #define ONE_WIRE_RESET_SEARCH 30
    #define ONE_WIRE_SEARCH 31
    #define ONE_WIRE_CRC8 32
    #define SET_PIN_MODE_STEPPER 33
    #define STEPPER_MOVE_TO 34
    #define STEPPER_MOVE 35
    #define STEPPER_RUN 36
    #define STEPPER_RUN_SPEED 37
    #define STEPPER_SET_MAX_SPEED 38
    #define STEPPER_SET_ACCELERATION 39
    #define STEPPER_SET_SPEED 40
    #define STEPPER_SET_CURRENT_POSITION 41
    #define STEPPER_RUN_SPEED_TO_POSITION 42
    #define STEPPER_STOP 43
    #define STEPPER_DISABLE_OUTPUTS 44
    #define STEPPER_ENABLE_OUTPUTS 45
    #define STEPPER_SET_MINIMUM_PULSE_WIDTH 46
    #define STEPPER_SET_ENABLE_PIN 47
    #define STEPPER_SET_3_PINS_INVERTED 48
    #define STEPPER_SET_4_PINS_INVERTED 49
    #define STEPPER_IS_RUNNING 50
    #define STEPPER_GET_CURRENT_POSITION 51
    #define STEPPER_GET_DISTANCE_TO_GO 52
    #define STEPPER_GET_TARGET_POSITION 53
    #define GET_FEATURES 54
    #define SONAR_DISABLE 55
    #define SONAR_ENABLE 56
    #define TONE = 57
    #define NO_TONE = 58
    #define GET_MAX_PINS = 59

    // maximum length of a command in bytes
    #define MAX_COMMAND_LENGTH 30

    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    /*                 Reporting Defines and Support                    */
    /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    // Reports sent to the client (THE NUMBER MUST BE IN ORDER AND WITHOUT ANY SKUP)

    #define DIGITAL_REPORT DIGITAL_WRITE
    #define ANALOG_REPORT ANALOG_WRITE
    #define FIRMWARE_REPORT 5
    #define I_AM_HERE 6
    #define SERVO_UNAVAILABLE 7
    #define I2C_TOO_FEW_BYTES_RCVD 8
    #define I2C_TOO_MANY_BYTES_RCVD 9
    #define I2C_READ_REPORT 10
    #define SONAR_DISTANCE 11
    #define DHT_REPORT 12
    #define SPI_REPORT 13
    #define ONE_WIRE_REPORT 14
    #define STEPPER_DISTANCE_TO_GO 15
    #define STEPPER_TARGET_POSITION 16
    #define STEPPER_CURRENT_POSITION 17
    #define STEPPER_RUNNING_REPORT 18
    #define STEPPER_RUN_COMPLETE_REPORT 19
    #define FEATURES 20
    #define NOT_IMPLEMENTED 21
    #define MAX_PIN_REPORT 22
    #define DEBUG_PRINT 99

    // Input pin reporting control sub commands (modify_reporting)
    #define REPORTING_DISABLE_ALL 0
    #define REPORTING_ANALOG_ENABLE 1
    #define REPORTING_DIGITAL_ENABLE 2
    #define REPORTING_ANALOG_DISABLE 3
    #define REPORTING_DIGITAL_DISABLE 4


    // DHT Report sub-types
    #define DHT_DATA 0
    #define DHT_READ_ERROR 1

    // Feature Masks And Storage

    #define ONEWIRE_FEATURE 0x01
    #define DHT_FEATURE 0x02
    #define STEPPERS_FEATURE 0x04
    #define SPI_FEATURE 0x08
    #define SERVO_FEATURE 0x10
    #define SONAR_FEATURE 0x20
    #define I2C_FEATURE 0x40

    // Pin mode definitions

    // The following are defined for arduino_firmetix (AT)
    #define AT_INPUT 0
    #define AT_OUTPUT 1
    #define AT_INPUT_PULLUP 2
    #define AT_ANALOG 3
    #define AT_ANALOG_OUTPUT 7
    #define AT_MODE_NOT_SET 255

    // maximum number of pins supported

    #ifdef ESP32
    #define MAX_DIGITAL_PINS_SUPPORTED 40
    #define MAX_ANALOG_PINS_SUPPORTED MAX_DIGITAL_PINS_SUPPORTED
    #elif ESP8266
    #define MAX_DIGITAL_PINS_SUPPORTED 16
    #define MAX_ANALOG_PINS_SUPPORTED MAX_DIGITAL_PINS_SUPPORTED
    #else
    #define MAX_DIGITAL_PINS_SUPPORTED NUM_DIGITAL_PINS
    #define MAX_ANALOG_PINS_SUPPORTED NUM_ANALOG_INPUTS
    #endif
};

#endif //_FIRMETIX_H_
