#include "mbed.h"
#include "uLCD_4DGL.h"
#include "stdio.h"
#include <cstdlib>
#include "math.h"
#include "Speaker.h"

DigitalOut myled(LED1);

Serial hc05(p9, p10); // master HC05 bluetooth
Serial pc(USBTX, USBRX); // serial pc connection for testing, *FOR DEBUG*

uLCD_4DGL uLCD(p28, p27, p11); // uLCD for lidar imaging
Speaker mySpeaker(p18); // speaker for collision detect noise

AnalogIn horz(p16); // left/right input from joystick
AnalogIn forward(p15); // forward input from joystick

volatile int mode = 0; // drive mode = 0 or collision detect mode = 1

volatile float Lspeed_tx = 0; // left speed for transmission
volatile float Rspeed_tx = 0; // right speed for transmission
volatile float Fspeed_tx = 0; // forward speed for transmission
volatile float Bspeed_tx = 0; // reverse speed for transmission

volatile float collision_dist; // collision distance read from lidar sensor on robot
volatile float collision_angle; // collision angle read from robot as input on servo controlling the lidar

// transmission of speeds from the master BT to the slave BT
void transmit() {
    
    while (1) {
        if (!mode) {
            
            hc05.printf("h%.3f,%.3f", Lspeed_tx, Rspeed_tx);
            Thread::wait(50);
            hc05.printf("v%.3f,%.3f", Fspeed_tx, Bspeed_tx);
            // pc.printf("H: %.3f,%.3f, V: %.3f,%.3f\r\n", Lspeed_tx, Rspeed_tx, Fspeed_tx, Bspeed_tx); // debug
        }
        
        Thread::wait(50);
    }
        
}

// reading of data from the joystick and storing it into the values transmitted
// by the HC05 on the controller
void joystick_read() {
    
    while (1) {
        
        if (!mode) {
            
            if (horz.read() > 0.65f) {
                Lspeed_tx = (horz.read() - 0.5f) * 2;
                Rspeed_tx = 0.0f;
            } else if (horz.read() < 0.45f) {
                Rspeed_tx = (0.60f - horz.read()) * 2;
                Lspeed_tx = 0.0f;
            } else {
                Lspeed_tx = 0.0f;
                Rspeed_tx = 0.0f;
            }
            
            if (forward.read() > 0.65f) {
                Fspeed_tx = (forward.read() - 0.5f) * 2;
                Bspeed_tx = 0.0f;
            } else if (forward < 0.45f) {
                Bspeed_tx = (0.55f - forward.read()) * 2;
                Fspeed_tx = 0.0f;
            } else {
                Fspeed_tx = 0.0f;
                Bspeed_tx = 0.0f;
            }
            
        } else {
            
            Lspeed_tx = 0;
            Rspeed_tx = 0;
            Fspeed_tx = 0;
            Bspeed_tx = 0;
            
        }
        
        Thread::wait(100);
    }
    
}

void collision_protocol() { /* Collision Avoidance Protocol */

    char collision_buffer[16];
    char dist_buffer[6];
    char angle_buffer[5];
    int a;
    bool good_buff;
    Timer ack_timer; /* Stop-And-Wait Acks */
    
    while (1) {
        if (hc05.readable()) {
            /* pc.printf("Reading HC05\r\n"); */
            memset(collision_buffer, NULL, 16);                    /* ZERO INIT */
            a = 0;
            while(a < 16) {
                if(hc05.readable()) {
                    collision_buffer[a] = hc05.getc();
                    if(collision_buffer[0] == 'r') { a++; }     /* START AT 'R' */
                }
            } pc.printf("%s\r\n", collision_buffer);
            
                 /* PSEUDO-CHECKSUM: CHECK ALL KNOWN POSITIONS TO VERIFY BUFFER */
            good_buff = ( (collision_buffer[0]  == 'r') && \
                          (collision_buffer[3]  == '1') && \
                          (collision_buffer[4]  == ',') && \
                          (collision_buffer[6]  == '.') && \
                          (collision_buffer[10] == ',') && \
                          (collision_buffer[12] == '.') );
            
            if(good_buff){ mode = 1; }                              /* SET MODE */
                            

            if (mode) {                         /* COLLISION AVOIDANCE PROTOCOL */
                /* pc.printf("Collision Avoidance Engaged\r\n"); */
                uLCD.cls();                                /* INIT RADAR SCREEN */
                uLCD.filled_circle(64, 64, 5, (RED+GREEN)); 
                mySpeaker.PlayNote(440, 1000, 0.8);                /* PLAY TONE */
                Thread::wait(1000);                     /* WAIT AN EXTRA SECOND */
                ack_timer.start();
                int i = 0;
                while(i < 60) {
                    good_buff = ( (collision_buffer[0]  == 'r') && \
                                  (collision_buffer[3]  == '1') && \
                                  (collision_buffer[4]  == ',') );
                                  
                    if(good_buff) {
                        i++;
                        hc05.printf("_ack_%02d", (i-1));           /* ACKNOWLEDGE */
                        /* pc.printf("_ack_%02d\r\n", i); */
                        ack_timer.reset();
                    }
                    
                    while(!hc05.readable()) { }            /* WAIT FOR NEW DATA */
                    memset(collision_buffer, NULL, 16);         /* RESET BUFFER */
                    // pc.printf("Buffer reset\r\n"); // debug
                    a = 0;
                    while(a < 16) {                              /* FILL BUFFER */
                        if(hc05.readable()) {
                            collision_buffer[a] = hc05.getc();
                            if(collision_buffer[0] == 'r') { a++; } 
                        }
                    } 
                 /* PSEUDO-CHECKSUM: CHECK ALL KNOWN POSITIONS TO VERIFY BUFFER */
                    good_buff = ( (collision_buffer[0]  == 'r') && \
                                  (collision_buffer[3]  == '1') && \
                                  (collision_buffer[4]  == ',') );
                    
                    if(good_buff) { /* ONLY USE IF GOOD BUFFER */
                        // pc.printf("Buffer: %s\r\n", collision_buffer); // debug
    
                        for (int j = 0; j < 5; j++) {
                            dist_buffer[j] = collision_buffer[j+6];
                        }
    
                        for (int j = 0; j < 5; j++) {
                            angle_buffer[j] = collision_buffer[j+11];
                        }
    
                        collision_dist = (float) atof(dist_buffer);
                        collision_angle = (float) atof(angle_buffer);
    
    
                        int x = (int) (64.0f + (collision_dist * 60 * cos(collision_angle)));
                        int y = (int) (64.0f + (collision_dist * 60 * sin(collision_angle)));
    
                        if (x >= 0 && y >= 0) {
                            uLCD.filled_circle(x, y, 2, GREEN);
                        }
                    } else if(ack_timer.read_ms() > 2000) { 
                        hc05.printf("_ack_%02d", (i-1));         /* ACKNOWLEDGE */
                        // pc.printf("Resending: _ack_%02d\r\n", i); // debug
                        ack_timer.reset();
                    }
                } hc05.printf("_ack_59");         /* SEND FINAL ACKNOWLEDGEMENT */
                Thread::wait(5000); 
                mode = 0;                                         /* RESET MODE */
            }
        }

        Thread::wait(100);

    }
}

    

int main() {
    /* init components */
    hc05.baud(115200);
    pc.baud(115200);
    uLCD.baudrate(115200);
    uLCD.background_color(BLACK);

    /* Spawn threads */
    Thread thread2(&transmit);
    Thread thread3(&joystick_read);
    Thread thread4(&collision_protocol);
    
    while(1) {  /* Physical indication from main thread */
        myled = 1;
        wait(0.2);
        myled = 0;
        wait(0.2);
    }
}












