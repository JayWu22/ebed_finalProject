#include "mbed.h"
#include "bbcar.h"
#include <cstdio>
#include <chrono>

#include "drivers/DigitalOut.h"
#include "erpc_simple_server.hpp"
#include "erpc_basic_codec.hpp"
#include "erpc_crc16.hpp"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "finalbbcar_server.h"

  #define CENTER_BASE 1500
  #define unitsFC 360                          // Units in a full circle
  #define dutyScale 1000                       // Scale duty cycle to 1/000ths
  #define dcMin 29                             // Minimum duty cycle
  #define dcMax 971                            // Maximum duty cycle
  #define q2min unitsFC/4                      // For checking if in 1st uadrant
  #define q3max q2min * 3                      // For checking if in 4th uadrant
  int start = 0;

BufferedSerial pc(USBTX, USBRX);
Ticker servo_ticker;
Ticker servo_feedback_ticker;
Thread eRPC(osPriorityNormal1);
Thread t;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

DigitalInOut pin8(D8);
PwmIn servo0_f(D9), servo1_f(D10);
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);

BusInOut qti_pin(D4,D5,D6,D7);

/** erpc infrastructure */
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

/** bbcar service */
bbcarService_service bbcar_service;

/** erpc declaration */
void bbcar_start(void) {
    
}

float bbcar_distance(void) {
    float distance1 = abs((float)(car.servo0.turns)*6.5*3.14);
    float distance2 = abs((float)(car.servo1.turns)*6.5*3.14);
    //printf("%lf\n", (float)((distance1+distance2)/2));
    return (float)((distance1+distance2)/2);
}

float bbcar_speed(void) {
    float initAngle0 = car.servo0.angle;
    float initAngle1 = car.servo1.angle;

    ThisThread::sleep_for(200ms);

    float length = ((float)car.servo0.angle - initAngle0) + ((float)car.servo1.angle - initAngle1)*6.50*3.14/360.0/2.0;
    float speed = length * 5;
    return speed;
}

void e_RPC () {
    // Initialize the rpc server
    uart_transport.setCrc16(&crc16);

    printf("Initializing server.\n");
    rpc_server.setTransport(&uart_transport);
    rpc_server.setCodecFactory(&basic_cf);
    rpc_server.setMessageBufferFactory(&dynamic_mbf);

    // Add the led service to the server
    printf("Adding bbcar server.\n");
    rpc_server.addService(&bbcar_service);

    // Run the server. This should never exit
    printf("Running server.\n");
    rpc_server.run();
}

int main() {
    pc.set_baud(9600);
    //t.start(callback(&queue, &EventQueue::dispatch_forever));
    eRPC.start(e_RPC);

    parallax_laserping  ping1(pin8);
    parallax_qti qti1(qti_pin);
    int pattern;

    int accept = 0;
    int direction = 0;
    int timer = 0;
    int timer2 = 0;
    int line = 0;
    //while (start != 1) {}
    car.goStraight(30);
    while(1) {
        pattern = (int)qti1;
        if (pattern == 0b1111) {
            accept++;
            line = 1;
        } else if (pattern == 0b0111) {
            direction = 1; // go right
            line = 0;
        } else if (pattern == 0b1110) {
            direction = 2; // go left
            line = 0;
        } else {
            line = 0;
        }

        if (direction == 1) {
            switch (pattern) {
                case 0b1000: car.turn(40, 0.001); break;
                case 0b1100: car.turn(40, 0.001); break;
                case 0b0100: car.turn(40, 0.001); break;
                case 0b1110: car.turn(40, 0.001); break; 
                case 0b0110: car.goStraight(40); break;
                case 0b0111: car.goStraight(40); break;
                case 0b0010: car.turn(40, -0.001); break;
                case 0b0011: car.turn(40, -0.001); break;
                case 0b0001: car.turn(40, -0.001); break;
                default: car.turn(55, -0.001);
            }
            timer++;
            timer2++;
        } else if (direction == 2) {
            switch (pattern) {
                case 0b1000: car.turn(40, 0.001); break;
                case 0b1100: car.turn(40, 0.001); break;
                case 0b0100: car.turn(40, 0.001); break;
                case 0b1110: car.goStraight(40); break; 
                case 0b0110: car.goStraight(40); break;
                case 0b0111: car.turn(40, -0.001); break;
                case 0b0010: car.turn(40, -0.001); break;
                case 0b0011: car.turn(40, -0.001); break;
                case 0b0001: car.turn(40, -0.001); break;
                default: car.turn(55, 0.001);
            }
            timer++;
            timer2++;
        } else {
            switch (pattern) {
                case 0b1000: car.turn(40, 0.001); break;
                case 0b1100: car.turn(40, 0.001); break;
                case 0b0100: car.turn(40, 0.001); break;
                case 0b1110: car.turn(40, 0.001); break;
                case 0b0110: car.goStraight(40); break;
                case 0b0111: car.turn(40, -0.001); break;
                case 0b0010: car.turn(40, -0.001); break;
                case 0b0011: car.turn(40, -0.001); break;
                case 0b0001: car.turn(40, -0.001); break;
                default: car.goStraight(40);
            }
            timer2++;
        }
        if  (accept >=3 && ping1 < 10) {
            int code[5];
            int Start = 0;
            car.stop();
            car.rotate(-40);
            ThisThread::sleep_for(2250ms);
            car.rotate(40);
            //printf("dis = %f\n", (float)ping1);   
          /*  while (Start != 1) { 
                if((float)ping1 < 15) {
                    Start = 1;
                }
            }*/
            for (int i = 0; i < 5; i++) {
                float value = 0;
                for (int j = 0; j < 10; j++) {
                    value += ping1;
                    ThisThread::sleep_for(45ms);
                }
                value /= 10;
                if(value < 15) code[i] = 1;
                else code[i] = 0;
                printf("%d\n", code[i]);    
            }
            accept = 0;
            if (code[1] == 0 || code[0] == 0) {
                car.rotate(-40);
                ThisThread::sleep_for(8s);
            } else if (code[3] == 0 || code[4] == 0) {
                car.rotate(40);
                ThisThread::sleep_for(3500ms);
            }
        }
        //printf("%d\n",pattern);
        if (timer > 230) { 
            direction = 0;
            timer = 0;
        }
        if (timer2 > 400) {
            accept = 0;
            timer2 = 0;
        }
        ThisThread::sleep_for(5ms);
    }
}
