#include "mbed.h"
#include "ultrasonic.h"
#include "motordriver.h"
#include "rtos.h"

#define STOP_DISTANCE 250
#define RADAR_SPEED 0.5
#define RADAR_WAIT 0.07
#define pi 3.14159265f

/* LEDS TO SHOW WHICH MODE ROBOT IS IN */
DigitalOut led_left(LED1);
DigitalOut led_center_left(LED2);
DigitalOut led_center_right(LED3);
DigitalOut led_right(LED4);

AnalogIn lightSensor(p20);
DigitalOut headlights(p30);

Serial hc05_slave(p9,p10);
Serial pc(USBTX, USBRX);

Motor LeftMotor(p21,p15,p16,1); //left motor
Motor RightMotor(p22,p17,p19,1); //right motor

volatile int mode = 0;

volatile float Lspeed_rx = 0;
volatile float Rspeed_rx = 0;
volatile float Fspeed_rx = 0;
volatile float Bspeed_rx = 0;

void collision_protocol (int distance);

ultrasonic usSensor(p6, p5, 0.050f, 1, &collision_protocol);

void collision_protocol (int distance) {
    
    if ((distance < STOP_DISTANCE) && (distance > 50)) {
        mode = 1;
        
        LeftMotor.speed(0); RightMotor.speed(0);
        Timer ack_timer; /* Stop-And-Wait Acks */
        // pc.printf("\r\nListening on Bluetooth...\r\n"); // debug
    
        float angle;
        
        hc05_slave.printf("rXX1,0.101,0.101");   /* SEND MAGIC COMMAND TO CONTROLLER */
        // pc.printf("\r\nSENT\r\n"); // debug
        
        LeftMotor.speed(-1); RightMotor.speed(-1);
        Thread::wait(400);                /* BACK UP TO AVOID BEING SET OFF AGAIN */
        LeftMotor.speed(0); RightMotor.speed(0);
        
        int x = 0; int dist = 0;
        int idx; char *ack;
        char response_buffer[25];
        memset(response_buffer, NULL, 25);                           /* ZERO INIT */
        ack_timer.start();
        while(x < 60) {
            
            idx = 0; ack = NULL;                               /* ACK --> NULLPTR */
            memset(response_buffer, NULL, 25);                    /* RESET BUFFER */
            
            while(hc05_slave.readable() && idx < 25){     /* FILL RESPONSE BUFFER */
                response_buffer[idx] = hc05_slave.getc();
                idx++;
            }  
                
            // pc.printf("\r\nRESPONSE: %s\r\n", response_buffer); // debug
        
            ack = strstr(response_buffer, "_ack_");                   /* FIND ACK */
        
            if(ack != NULL) {                                     /* ACK RECIEVED */
                pc.printf("Acknowledged - Sending reading %d\r\n",x);
                angle = ((2*pi)/60) * x;    /* add pi/2 */
                dist = usSensor.getCurrentDistance(); 
                if(dist >= 5000) { dist = 4999; }
                // pc.printf("1,%.3f,%.3f\r\n", (((float) dist)/500), angle); // debug
                hc05_slave.printf("r%02d1,%02.3f,%.3f", x,(((float) dist)/500), angle);
                
                LeftMotor.speed(RADAR_SPEED); RightMotor.speed(-1*RADAR_SPEED);
                Thread::wait(RADAR_WAIT * 1000);    /* TURN TO NEXT READING ANGLE */
                LeftMotor.speed(0); RightMotor.speed(0);
                
                ack_timer.reset(); x++;
            } else {                                              /* WAIT FOR ACK */
                pc.printf("Waiting for ack...\r\n");
                
                if(ack_timer.read_ms() > 2000) {  /* RETRANSMIT IF ACK NOT RECV'D */
                    hc05_slave.printf("r%02d1,%02.3f,%.3f", x,(((float) dist)/500), angle);
                    ack_timer.reset(); //  pc.printf("Retransmitting...\r\n"); // debug
                } led_center_left = led_center_right = 1;
            }      
            led_center_left = led_center_right = 1;
            Thread::wait(100);
            led_center_left = led_center_right = 0;
        } mode = 0;                                                /* RESET MODE */
    }
}

void coll_start () {
    
    usSensor.startUpdates();
    Thread::wait(2000);
    while(1) {
        if (!mode) {
            usSensor.checkDistance();
            // pc.printf("%d", dist); // debug
            Thread::wait(50);
        }
    }
}

void headlight_switch () {
    
    headlights = 0;
    
    while (1) {
        if (lightSensor.read() < 0.09) {
            headlights = 1;
        } else if (lightSensor > 0.105) {
            headlights = 0;
        }
    }
}

void read_speeds () {
    
    char speed_buffer[11];
    char Lspeed_buff[5];
    char Rspeed_buff[5];
    char Fspeed_buff[5];
    char Bspeed_buff[5];
    char c;
    
    while (1) {
        
        if (hc05_slave.readable() && !mode) {
            
            memset(speed_buffer, NULL, 11);
            
            c = hc05_slave.getc();
            if((c == 'v') || (c == 'h')) {
                int i = 0;
                while (i < 11){
                    if(hc05_slave.readable()) {
                        speed_buffer[i] = hc05_slave.getc();
                        i++;
                    }
                }
                
                if (c == 'h') {
                    
                    for (int j = 0; j < 5; j++) {
                        Lspeed_buff[j] = speed_buffer[j];
                    }
                
                    for (int j = 0; j < 5; j++) {
                        Rspeed_buff[j] = speed_buffer[j+6];
                    }
                    
                    Lspeed_rx = atof(Lspeed_buff);
                    Rspeed_rx = atof(Rspeed_buff);
                    
                } else if (c == 'v') {
                
                    for (int j = 0; j < 5; j++) {
                        Fspeed_buff[j] = speed_buffer[j];
                    }
                    
                    for (int j = 0; j < 5; j++) {
                        Bspeed_buff[j] = speed_buffer[j+6];
                    }
                    
                    Fspeed_rx = atof(Fspeed_buff);
                    Bspeed_rx = atof(Bspeed_buff);
                    
                }
                
                if (Fspeed_rx > 0.0f) {
                    
                    // pc.printf("forward\r\n");    // debug
                    // pc.printf("LF: %f, RF: %f\r\n", (Fspeed_rx * 0.5f + Rspeed_rx * 0.4), Fspeed_rx * 0.5f + Lspeed_rx * 0.4f);
                    LeftMotor.speed(Fspeed_rx * 0.6f + Rspeed_rx * 0.4f);
                    RightMotor.speed(Fspeed_rx * 0.6f + Lspeed_rx * 0.4f);
                    
                } else if (Bspeed_rx > 0.0f) {
                    
                    // pc.printf("reverse\r\n");    // debug
                    // pc.printf("LF: %f, RF: %f\r\n", 0-(Bspeed_rx * 0.5f + Rspeed_rx * 0.4), 0-(Bspeed_rx * 0.5f + Lspeed_rx * 0.4f));
                    LeftMotor.speed(0-(Bspeed_rx * 0.5f + Rspeed_rx * 0.4f));
                    RightMotor.speed(0-(Bspeed_rx * 0.5f + Lspeed_rx * 0.4f));
                    
                } else {
                    
                    // pc.printf("turn only\r\n");   // debug
                    // pc.printf("LF: %f, RF: %f\r\n", Rspeed_rx * 0.4f, Lspeed_rx * 0.4f);
                    LeftMotor.speed(Rspeed_rx * 0.4f);
                    RightMotor.speed(Lspeed_rx * 0.4f);
                    
                }
            }
        }
    }
}

int main() {
    /* init components */
    hc05_slave.baud(115200);
    pc.baud(115200);
    
    /* Spawn threads */
    Thread thread2(&read_speeds);
    Thread thread3(&coll_start);
    Thread thread4(&headlight_switch);
    
    while(1) {
        if(!mode) { /* Physical indication from main thread */
            led_left = led_right = 1;
            wait(0.2);
            led_left = led_right = 0;
            wait(0.2);
        }
    }
}
