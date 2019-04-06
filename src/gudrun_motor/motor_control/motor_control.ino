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

//Flag that helps us restart counter when we first find header byte
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
  //Check if there is any data available to read
  if(Serial.available() > 0){
    //read only one byte at a time
    uint8_t c = Serial.read();
    
    //Check if header is found
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
    
    //store received byte, increase readCounter
    buffer[readCounter] = c;
    // Serial.print("buffer["); Serial.print(readCounter); Serial.print("] = "); Serial.println(buffer[readCounter]);
    readCounter++;


    //prior overflow, we have to restart readCounter
    if(readCounter >= bufferSize){
      readCounter = 0;
      
      //if header was found
      if(isHeader){

        //get checksum value from buffer's last value, according to defined protocol
        uint8_t checksumValue = buffer[bufferSize-1];

        //perform checksum validation, it's optional but really suggested
        Serial.print("buffer = [");
        for(int j=0; j<bufferSize; j++) {
          Serial.print(buffer[j], DEC);
          Serial.print(", ");
        }
        Serial.println("]");
        if(verifyChecksum(checksumValue)){
          Serial.println("checksum_ok");

          steering.write(buffer[1]);

          float throttle_us = 1000 + 1000. * ((float) buffer[2]) / 180.;
          Serial.print("throttle microseconds: "); Serial.println((int) throttle_us);
          throttle.writeMicroseconds((int) throttle_us);
          // throttle.write(buffer[2]);


        } else {
          Serial.println("checksum_bad");
        }
        
        //restart header flag
        isHeader = 0;
        firstTimeHeader = 0;
      }
    }
  }
}

//This a common checksum validation method
//We perform a sum of all bytes, except the one that corresponds to the original
//checksum value. After summing we need to AND the result to a byte value.
uint8_t verifyChecksum(uint8_t originalResult){
  uint8_t result = 0;
  uint16_t sum = 0;
  
  for(uint8_t i = 0; i < (bufferSize - 1); i++){
    sum += buffer[i];
  }
  result = sum & 0xFF;

  Serial.print("Got checksum: "); Serial.println(result);
  
  if(originalResult == result){
     return 1;
  }else{
     return 0;
  }
}
