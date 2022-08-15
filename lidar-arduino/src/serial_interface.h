#define REV_COMPLETED_FLAG 0x01
#define SET_MIRROR_SPEED 0x02
#define START_COMMAND 0x03
#define STOP_COMMAND 0x04
#define ERROR_MOTOR 0x05
#define ERROR_CHECKSUM 0x06
#define SUCCESS_COMMAND 0x07
#define ERROR_UNKNOWN_COMMAND 0x08
#define ERROR_COMMAND_ALREADY_EFFECTIVE 0x09
#define ERROR_NO_VELOCITY 0x0A
#define ERROR_TIMEOUT 0x0B

#define WHOAMI 0xFF
#define ID 0xFE

extern double mirror_velocity; // velocity of the mirror

void handle_serial_requests();