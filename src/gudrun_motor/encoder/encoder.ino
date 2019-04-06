volatile long count;
// volatile long millis_last_count;
// volatile double counts_per_second;
// unsigned long timeold;

// double dt, rpm, wheel_revolutions, wheel_revolutions_old, last_printed_wheel_revolutions;

const int PIN_A = 2;
const int PIN_A_INTNUM = 1;
const int PIN_B = 3;
const int PIN_B_INTNUM = 2;


void magnet_detect()//This function is called whenever a magnet/interrupt is detected by the arduino
{
	count++;
	// long t = millis();
 	// counts_per_second = 1000. / (t - millis_last_count);
	// millis_last_count = t;
	// Serial.print("    count = ");

	// Serial.println(count);

	// Serial.print("    motor_cps = ");
	// Serial.println(counts_per_second);
	
	// Serial.print("Saw movement on pin A: ");
	// Serial.println(count);
}

// void magnet_detect_b()
// {
// 	Serial.print("Saw movement on pin B: ");
// 	Serial.println(count);
// }

void setup()
{
	Serial.begin(115200);

	//Initialize the intterrupt pin (Arduino digital pin 2)
	pinMode(PIN_A, INPUT_PULLUP);
	pinMode(PIN_B, INPUT_PULLUP);
	attachInterrupt(PIN_A_INTNUM, magnet_detect, CHANGE);
	// attachInterrupt(PIN_B_INTNUM, magnet_detect_b, CHANGE);
	count = 0;
	// rpm = 0;
	// timeold = 0;
	// wheel_revolutions = 0;
	// wheel_revolutions_old = 0;
	// millis_last_count = millis();
}
void loop()//Measure RPM
{
	// ;

	if ( millis() % 10 == 0) {
		Serial.println(count);
		// wheel_revolutions = (double) count / CPR_WHEEL;
		// if (wheel_revolutions != last_printed_wheel_revolutions) {
		// 	dt = (millis() - timeold) / 1000. * 60.;
		// 	rpm = (wheel_revolutions - wheel_revolutions_old) / dt;
		
		// 	timeold = millis();
		// 	wheel_revolutions_old = wheel_revolutions;

		// 	Serial.print("RPM = ");
		// 	Serial.println(rpm, DEC);
		// 	last_printed_wheel_revolutions = wheel_revolutions;
		// }
	}
}
