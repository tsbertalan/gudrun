volatile long count;
volatile byte last_symbol, symbol, a, b;

const int PIN_A = 2;
const int PIN_B = 3;

// There are two distinct seminibble sequences;
// one for forward, one reverse.
// Current byte     ->       0b00, 0b01, 0b10, 0b11
// Previous byte:
const byte fwd_sources[4] = {0b10, 0b00, 0b11, 0b01};
const byte rev_sources[4] = {0b01, 0b11, 0b00, 0b10};

void isr() {
	// We enter here whenever a rising or falling edge is seen on either channel.

	a = digitalRead(PIN_A);
	b = digitalRead(PIN_B);

	symbol = b + (a << 1);

	if(fwd_sources[symbol] == last_symbol) {
		count++;
	}

	if(rev_sources[symbol] == last_symbol) {
		count--;
	}

	last_symbol = symbol;	
}

void setup() {
	Serial.begin(115200);

	// Initialize the interrupt pins (these need to be interrupt-capable).
	pinMode(PIN_A, INPUT_PULLUP);
	pinMode(PIN_B, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_A), isr, CHANGE);
	attachInterrupt(digitalPinToInterrupt(PIN_B), isr, CHANGE);

	count = 0;
}
void loop() {
	if ( millis() % 10 == 0) {
		Serial.println(count);
	}
}
