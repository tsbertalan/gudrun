# Gudrun

<a href="headshot.jpg"><img src="headshot.jpg" width="24%" /></a>
<img src="1_small.gif" width="24%" />
<img src="2_small.gif" width="24%" />
<a href="https://youtu.be/KhBlflgKe1Q?t=81"><img src="teb_demo.gif" width="24%" /></a>

This robot will be a variant of the [Donkey car](http://www.donkeycar.com/), probably using the [recommended chassis](https://hobbyking.com/en_us/trooper-pro-4x4-1-10-brushless-sct-arr.html). 
However, instead of controlling it with a Raspberry Pi, I'll use [a computer I built last year](https://pcpartpicker.com/user/tsbertalan/saved/#view=dk9GXL) (mini-ITX, I think) for a different purpose. I might replace the motherboard with a slim-mini-ITX, a standard which includes a 19VDC power jack. 
Otherwise I'd need something like [this](https://www.amazon.com/dp/B005TWE6B8/?coliid=I3T66Y7O6B2HJK&colid=3LRY6AZNFBVCM&psc=0&ref_=lv_ov_lig_dp_it) to provide motherboard power.

Either way, I'll probably need to use a boost converter (which I already have), and I think it should draw about 130W max (according to the estimate from pcpartpicker on the page where you select a power supply). 
[This battery](https://hobbyking.com/en_us/multistar-high-capacity-4s-10000mah-multi-rotor-lipo-pack.html) which is currently in Gunnar should be able to handle 100 amps according to [this calculator](https://www.kritikalmass.net/battery-calculator/index.php), so I think this should be fine for about an hour of use--certainly a half-hour. 

In a later iteration, I might use the more power-efficient [Nvidia TX2](https://devtalk.nvidia.com/default/topic/1024102/jetson-tx2/jetson-tx2-power-consumption/), but, for this build, I want to minimize the specialness of the computer as much as possible, so that everything is just standard Ubuntu. (The TX2, like the Raspberry Pi, runs a custom linux with an ARM CPU, instead of a "normal" x86_64 CPU, meaning installing software is often harder.)

[Gudrun](https://en.wikipedia.org/wiki/Gudrun) will be the successor to two previous builds of mine, [Hogni](https://github.com/tsbertalan/hogni) and [Gunnar](https://github.com/tsbertalan/gunnar), all three of whom were mythologically siblings. (Also, note to self, if I want to continue this naming scheme, there's a [good list at the bottom](https://en.wikipedia.org/wiki/Gudrun#Family_relations) of that article.)

