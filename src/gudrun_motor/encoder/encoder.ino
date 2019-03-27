volatile long count;
// volatile long millis_last_count;
// volatile double counts_per_second;
// unsigned long timeold;

// double dt, rpm, wheel_revolutions, wheel_revolutions_old, last_printed_wheel_revolutions;

const int PIN_A = 2;
const int PIN_A_INTNUM = 1;
// const int CPR_WHEEL = 360; //=48*90/12 gear ratio is 90:12 and motor encoder CPR is 48.


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
}

void setup()
{
	Serial.begin(115200);

	//Initialize the intterrupt pin (Arduino digital pin 2)
	pinMode(PIN_A, INPUT_PULLUP);
//	pinMode(PIN_A, INPUT);
	attachInterrupt(PIN_A_INTNUM, magnet_detect, CHANGE);
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
