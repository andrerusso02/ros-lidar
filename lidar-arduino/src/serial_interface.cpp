#include <Arduino.h>
#include "serial_interface.h"
#include "hardware.h"

double mirror_velocity = 0.0;

void handle_serial_requests(){
    if(Serial.available() > 0){
        uint8_t c = Serial.read();
        if(c == SET_MIRROR_SPEED){
            byte buff[3];
            int ibuff = 0;
            unsigned int timeout = 20;
            unsigned long t_start = millis();
            while(ibuff < 3){
                if(Serial.available() > 0 || (millis() - t_start) > timeout){
                    buff[ibuff] = Serial.read();
                    ibuff++;
                }
            }
            if(ibuff != 3){
                Serial.write(ERROR_TIMEOUT);
                return;
            }
            byte VEL_H = buff[0];
            byte VEL_L = buff[1];
            byte checksum = buff[2];
            if(((VEL_H + VEL_L) & 0xFF)  != checksum){
                Serial.write(ERROR_CHECKSUM);
            }
            else{
                mirror_velocity = VEL_H + (double)VEL_L/(double)0x100;
                //set_mirror_speed(mirror_velocity);
                Serial.write(SUCCESS_COMMAND);
            }

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