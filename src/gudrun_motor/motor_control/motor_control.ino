#include <Servo.h>

Servo steering, throttle;
int incomingByte = 0;
int angle0 = 0;
int angle1 = 0;

const uint8_t header = 0x7E;
const uint8_t bufferSize = 4;

const int THROTTLE_NEUTRAL = 91;

uint8_t buffer[bufferSize];
uint8_t readCounter;
uint8_t isHeader;

// Flag that helps us restart the counter when we first see the header byte:
uint8_t firstTimeHeader; 

void setup(){
  while(!Serial);
  Serial.begin(115200);

  steering.attach(2);
  throttle.attach(9);
  throttle.write(THROTTLE_NEUTRAL);
  delay(300);
  
  readCounter = 0;
  isHeader = 0;
  firstTimeHeader = 0;
}

void loop(){
  // If there's data to read, we do something; else we spin.
  if(Serial.available() > 0){

    // Read only one byte at a time.
    uint8_t c = Serial.read();
    
    // Check if header is found.
    if(c == header){
      //We must consider that we may sometimes receive unformatted data, and
      //given the case we must ignore it and restart our reading code.
      //If it's the first time we find the header, we restart readCounter
      //indicating that data is coming.
      //It's possible the header appears again as a data byte. That's why
      //this conditional is implemented, so that we don't restart readCounter
      //and corrupt the data. 
      if(!firstTimeHeader){
        isHeader = 1;
        readCounter = 0;
        firstTimeHeader = 1;
      }
    }
    
    // Store received byte, and update the pointer index.
    buffer[readCounter] = c;
    readCounter++;

    // We either overflowed due to bad reading,
    // or finished reading a full packet.
    if(readCounter >= bufferSize){
      readCounter = 0;
      
      // If header was previously seen, we can consider reading a packet.
      if(isHeader){

        // Get checksum value from buffer's last value, according to defined protocol
        uint8_t checksumValue = buffer[bufferSize-1];

        // Perform checksum validation.
        if(verifyChecksum(checksumValue)){

          steering.write(buffer[1]);

          // TODO: Use writeMicroseconds everywhere. This will require sending two bytes per datum to cover the 1000-microsecond range between 1 ms and 2 ms. So, [HEADER, angle_byte1, angle_byte2, throttle_byte1, throttle_byte2, checksum].
          // float throttle_us = 1000 + 1000. * ((float) buffer[2]) / 180.;
          // Serial.print("throttle microseconds: "); Serial.println((int) throttle_us);
          // throttle.writeMicroseconds((int) throttle_us);
          throttle.write(buffer[2]);


        } else {
          Serial.println("csbad");
        }
        
        //restart header flag
        isHeader = 0;
        firstTimeHeader = 0;
      }
    }
  }
}

// Checksum validation
//
// We perform a sum of all bytes, except the one that corresponds to the original
// checksum value. After summing we AND the result with 0b11111111=0xFF 
// to get only the least-significant 8 bits.
uint8_t verifyChecksum(uint8_t originalResult){
  uint8_t result = 0;
  uint16_t sum = 0;
  
  for(uint8_t i = 0; i < (bufferSize - 1); i++){
    sum += buffer[i];
  }
  result = sum & 0xFF;
  
  if(originalResult == result){
     return 1;
  }else{
     return 0;
  }
}
