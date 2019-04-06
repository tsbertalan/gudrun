**<u>Question</u>**

I'm trying to control the brushed motor in a Traxxas Slash RC car with the built-in XL5 ESC using the Servo library on Arduino. I previously had success controlling this car with a Pololu Micro Maestro, but it died on me, and so, while waiting for a replacement to arrive, I thought I'd give a more manual approach a try.

The problem is, the motor moves *way* too fast. My understanding is that, for this ESC (which allows for reverse driving), 90 degrees / 1500 ms should be the neutral point, with anything above this leading to progressively faster forward speed; and, below, reverse.
But, while it seems there is some speed modulation, the motor just abruptly begins with a very high speed at an angle (`Serial.write(95)`) of 95 or so, and then gets even faster from there.

A minimal version of my code is pasted below; my circuit has the signal pins of the steering and ESC servo connectors (white wires) attached to pins 2 and 9, respectively on a Pro Micro, their two 5V pins connected to each other (since the ESC supplies its own 5V), and their grounds connected to the Arduino's ground pins.

While it's possible that I need a special arming sequence to get this ESC to work properly (I do operate in the three-blink "training" mode, which is supposed to be slower), I would expect that problem to manifest as no motion at all, not too *much* motion.

I've tried powering the ESC with the original 9.1V battery (nominal 8.4V), and with a 8.4V regulated supply attached to a ~15V battery.

What could I be doing wrong here?

```c++
#include <Servo.h>

Servo throttle;

void setup(){

    Serial.begin(115200);

    throttle.attach(9);
    // throttle.write(91);
    // delay(300);

    int first = 90;
    for(int angle=first; angle < first+20; angle++) {
        Serial.println(angle);
        throttle.write(angle);
        delay(10);
    }

    // This doesn't work any better.    
    // int first = 1500;
    // for(int us=first; us < first+10; us++) {
    //     Serial.println(us);
    //     throttle.writeMicroseconds(us);
    //     delay(50);
    // }

    throttle.write(90);
}

void loop() {
}
```

<u>**Answer:**</u>

Per @jsotola 's suggestion, I looked more closely at the [manual][1] (see the section "XK-5 Setup Programming"):

```
XL-5 Setup Programming (Calibrating your ESC and transmitter)

Read through all of the programming steps before you begin. If you get lost during programming or receive unexpected results, simply unplug the battery, wait a few seconds, plug the battery pack in, and start over.

1. Disconnect one of the motor wires between the XL-5 and the motor. This is a precaution to prevent runaway when the speed control is turned on before it is programmed.

2. Connect a fully charged battery pack to the XL-5.

3. Turn on the transmitter (with the throttle at neutral).

4. Press and hold the EZ-Set button (A). The LED will first turn green and then red. Release the button.

5. When the LED blinks RED ONCE, pull the throttle trigger to the full throttle position and hold it there (B).

6. When the LED blinks RED TWICE, push the throttle trigger to the full reverse and hold it there (C).

7. When the LED blinks GREEN ONCE, programming is complete. The LED will then shine green or red (depending on LowVoltage Detection setting), indicating the XL-5 is on and at neutral (D).
```

I wrote a short program to send specified servo angles in [0, 180] interactively. The manual says to apply "full throttle" and "full reverse", but I wasn't exactly sure what these translated to in terms of servo angle/pulse width milliseconds. I gather that these are often given as 45° and 135° (1.25 ms and 1.75 ms), respectively, but that setting the limits to [0, 180] during this training period also worked. It's valuable to get the full range, so I can get finer gradations in speed. Since I don't expect to ever reach full speed in my application, and will only use the near-90 part of the range, this will be particularly helpful.

I'm still perplexed about what caused the original problem--the Pololu servo controller never needed this tuning process. Perhaps the limits were set to [45, 135], and I mistakenly sent 180, which put the ESC into some state not described in the manual. And I don't know whether I'll need to repeat this in the future, or 
whether my new [0, 180] limits will prevent the above mistake from being a problem. For that matter, I don't know how non-volatile the non-volatile memory is on the ESC in which these limits are stored. Does it require periodic battery power to refresh?

Finally, while it might be even more particular to the XL-5 ESC and my power supply setup, another useful note is that the low-voltage detection mode might prevent movement independent of the setup process.


[1]: https://traxxas.com/sites/default/files/58024-OM-EN-R01.pdf    "Traxxas Slash manual, XL-5 version"

