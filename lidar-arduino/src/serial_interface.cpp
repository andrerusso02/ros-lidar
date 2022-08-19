#include <Arduino.h>
#include "serial_interface.h"
#include "hardware.h"

double mirror_velocity = 0.0;

int read_float(float *f) {
    byte buff[5];
    int ibuff = 0;
    unsigned int timeout = 40;
    unsigned long t_start = millis();
    while(ibuff < 5){
        if(Serial.available() > 0 || (millis() - t_start) > timeout){
            buff[ibuff] = Serial.read();
            ibuff++;
        }
    }
    if(ibuff != 5){
        return ERROR_TIMEOUT;
    }
    if(((buff[0] + buff[1] + buff[2] + buff[3]) & 0xFF)  != buff[4]){ // checksum : last byte of sum
        return ERROR_CHECKSUM;
    }
    else
    {
        memcpy(f, buff, 4);
        return SUCCESS_COMMAND;
    }
}

void handle_serial_requests(){
    if(Serial.available() > 0){
        uint8_t c = Serial.read();
        if(c == SET_MIRROR_SPEED){
            float f;
            byte res = read_float(&f); // for test, send 0x2 0x0 0x0 0x80 0x3f 0xbf -> 1.0 rad/s
            if(res == SUCCESS_COMMAND){
                mirror_velocity = f;
                
            }
            Serial.write(res);
        }
        else if(c == START_COMMAND){
            if(!running && mirror_velocity != 0.0){
                if(start()==0)
                    Serial.write(SUCCESS_COMMAND);
                else
                    Serial.write(ERROR_MOTOR);
            }
            else if(mirror_velocity == 0.0) Serial.write(ERROR_NO_VELOCITY);
            else Serial.write(ERROR_COMMAND_ALREADY_EFFECTIVE);

        }
        else if(c == STOP_COMMAND){
            if(running){
                stop();
                Serial.write(SUCCESS_COMMAND);
            }
            else Serial.write(ERROR_COMMAND_ALREADY_EFFECTIVE);
        }
        else if(c == WHOAMI){
            Serial.write(ID);
        }
        else{
            Serial.write(ERROR_UNKNOWN_COMMAND);
        }
    }
}