const int SENSOR_COUNT = 2;

// defines pins numbers
const int trigPins[2] = {9, 11};
const int echoPins[2] = {10, 12};

// defines variables
long duration;
double distance;

int trigPin, echoPin;

void setup() {
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        trigPin = trigPins[i];
        echoPin = echoPins[i];
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    Serial.begin(9600);
  
}

void loop() {

    for (int i = 0; i < SENSOR_COUNT; i++) {

        trigPin = trigPins[i];
        echoPin = echoPins[i];

        // Clears the trigPin
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);

        // Send a pulse.
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        // Listen for the pulse;
        // return the duration in microseconds.
        duration = pulseIn(echoPin, HIGH);

        // Calculating the distance
        distance = duration * 0.034 / 2;
        Serial.print(distance);
        if( i < SENSOR_COUNT-1)
            Serial.print(",");
    }

    Serial.println("");    
}
