/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) : 
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send min throttle
 *     5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
 *
 * @author lobodol <grobodol@gmail.com>
 */
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1300 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 1650 // Maximum pulse length in µs
#define MED_PULSE_LENGTH 1500 // Medium
#define MIN_PULSE_ACTUAL 1000
#define MAX_PULSE_ACTUAL 2000
// ---------------------------------------------------------------------------
Servo mot;
// ---------------------------------------------------------------------------

/**
 * Initialisation routine
 */
void setup() {
    Serial.begin(9600);
    
    mot.attach(5, MIN_PULSE_ACTUAL, MAX_PULSE_ACTUAL);
    
    displayInstructions();
}

/**
 * Main function
 */
void loop() {

  char data;
    if (Serial.available()) {
        data = Serial.read();

        // ASCII table: https://www.arduino.cc/en/Tutorial/ASCIITable

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      mot.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // -
            case 45 : Serial.println("Sending medium throttle");
                      mot.writeMicroseconds(MED_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      mot.writeMicroseconds(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }
    

}

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = MED_PULSE_LENGTH  ; i <= MAX_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        mot.writeMicroseconds(i);
        
        delay(200);
    }

     for (int i = MAX_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);

        // Need a short negative kick to go into reverse mode.
        if(i==MED_PULSE_LENGTH) {
          mot.writeMicroseconds(i-100);
          delay(42);
          mot.writeMicroseconds(i);
        }
        
        mot.writeMicroseconds(i);
        
        delay(200);
    }

    for (int i = MIN_PULSE_LENGTH  ; i <= MED_PULSE_LENGTH; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        mot.writeMicroseconds(i);
        
        delay(200);
    }
    
    Serial.println("STOP");
    mot.writeMicroseconds(MED_PULSE_LENGTH);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t- : Send medium throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}

