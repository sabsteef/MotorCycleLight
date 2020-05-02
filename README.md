# VFR 1200F LED lights
Make custom LED modification to your bike.

First of, this my mod for my own bike. Your free to use it. but it's for your own risk. 

## So how does it look

[![Demo lichts](https://img.youtube.com/vi/mF-TudgtvgI/0.jpg)](https://www.youtube.com/watch?v=mF-TudgtvgI)

## What parts do you need:

#### Arduino:
I used a Nano
https://www.hobbyelectronica.nl/product/arduino-nano-compatible-ch340/

#### LED:
I used the WS2812B RGB led strips (144 leds on a strip)
https://nl.aliexpress.com/item/33023954212.html?spm=a2g0s.9042311.0.0.55c64c4dgj55Hw
about 40 in the mirrors and 100 in the tail.

#### Switch 3x:
LM317 DC-DC Converter Buck Step Down
https://nl.aliexpress.com/item/32230123301.html?spm=a2g0s.9042311.0.0.27424c4dBOdcSY
leds and nano are 5v

#### Power Converter to 5v
DC-DC Step-up-down Buck-Boost Converter XL6009 4A
https://nl.aliexpress.com/item/32788804655.html?spm=a2g0o.productlist.0.0.5d9af51b7OeitG&algo_pvid=80fc074c-2700-4d11-9302-89aec478d151&algo_expid=80fc074c-2700-4d11-9302-89aec478d151-3&btsid=0b0a187b15884130200895767eb79d&ws_ab_test=searchweb0_0,searchweb201602_,searchweb201603_
because the LEDS are 5v

#### 3d printed cases and parts:
https://www.tinkercad.com/users/7O7vk3583zk-sabsteef

#### Nano Case
https://www.thingiverse.com/thing:952188

### optional:
I did not want to cut in to my original parts. So I bought used parts.   
#### mirrors:
U can buy them from aliexpress as an imitation set. Or buy the blinker as a used part:
https://hotparts.nl/en/home/5731-blinker-mirror-l-honda-vfr-1200-f-2009-2017-vfr1200f-vfr1200.html 

#### taillight:
I bought a used part of a 2013 model.
https://www.motorparts-online.com/nl/honda/vfr-1200/2010-2013/achterlicht/201331259

what i did to make it:
#### Mirrors:
open them up. gut the original light. and mount the Led stripts and connect them to 1 strip.
( Images here)

#### tail light:
open the unit. ( in oven 105 degrees for 10 min)
3d print the support bars glue the together.
( images here)

glue the in the blinker location and glue LED strips on them. en connect the stripts together.
if needed paint the stripts grey

#### Powersupply
because i cannot find 12v LEDs that can change color i used 5v LEDs. and i need a step down converter.
print a case for it and connect the power suppy to the Nano and LEDs.
connect the power in ( Vin) to the tail light 12v. 
( images here)


#### Switchs
because the Nano and LEDs cannot handel 12v u use stepdowns as a switch. Solder a 1000k resistor on the output + and -
we have 3. 1 for left blinker 1 for the right blinker and 1 for the brake.
so connect the acordingly to the 12 volts of them. and connect the 5V to the nano
Brake pin D5
Blinker left pin  D6
blinker right pin D4
and print the Case for the switch
( images here)


#### Nano
Upload the provided software. and connect the pins to the LEDS
pin 11 to blinker Left
pin 12 to Blinker Right
pin 10 to Brake

PS the code i provided is coded how i sticked the LEDS in the light units. it can be difrend then yours.

( images here)



## whats Next?!
Im not finished yet. changed will be posted here.
