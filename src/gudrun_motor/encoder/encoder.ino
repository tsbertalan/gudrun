volatile long count;
volatile byte last_symbol;

// volatile long millis_last_count;
// volatile double counts_per_second;
// unsigned long timeold;

// double dt, rpm, wheel_revolutions, wheel_revolutions_old, last_printed_wheel_revolutions;

const int PIN_A = 2;
const int PIN_B = 3;

const byte fwd_sources[4] = {2, 0, 3, 1};
const byte rev_sources[4] = {1, 3, 0, 2};

void isr() {

	byte a = digitalRead(PIN_A);
	byte b = digitalRead(PIN_B);

	// Serial.print("0b");
	// Serial.print(a);
	// Serial.print(b);

	byte symbol = b + (a << 1);
	// Serial.print(" = "); Serial.println(symbol, DEC);

	if(fwd_sources[symbol] == last_symbol) {
		// Serial.println("Forward.");
		count++;
	}

	if(rev_sources[symbol] == last_symbol) {
		// Serial.println("Reverse.");
		count--;
	}

	// Serial.println();

	last_symbol = symbol;	
}

void setup() {
	Serial.begin(115200);

	//Initialize the intterrupt pin (Arduino digital pin 2)
	pinMode(PIN_A, INPUT_PULLUP);
	pinMode(PIN_B, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(PIN_A), isr, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_B), isr, CHANGE);

	count = 0;
}
void loop() {
	// ;

	if ( millis() % 10 == 0) {
		Serial.println(count);
	}
}
