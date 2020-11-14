# ESP32 WS2812b Clock
Experimenting with this incredible low-priced WS2812b 241-LED disk.

I found that feeding it enough power required some work. When the inner rings are starved for power they behave erratically. The wiring shipped with the 241-LED disk, connected daisy chain, doesn't work reliably with the breadboard power scheme I used in the past for the 93-LED disks. I soldered in the sort of bus bar connections I used in the graduation cap project and now all the LEDs behave reliably.

Dimensions of the disk are here: https://github.com/Mark-MDO47/ESP32WS2812bClock/blob/main/images/LED_241_Dimensions.jpg

With 241 LEDs, the RAM requirements exceed my old standby of Arduino Nano.  I chose the ESP32.


# Parts List
Parts List so far

| Num | CostEach | CostTotal | Description | Source |
| --- | --- | --- | --- | --- |
| 1 | $29.99 | $29.99 | 241 LEDs 9 Rings WS2812B 5050 RGB LED | https://smile.amazon.com/gp/product/B083VWVP3J/ |
| 1 | $5.16 | $5.16 | ESP32 ESP-32S CP2102 NodeMCU-32S ESP-WROOM-32 WiFi Unassembled | https://smile.amazon.com/gp/product/B08DQQ8CBP/ |
| 0 | $6.00 | $0.00 | ESP32 ESP-32S CP2102 NodeMCU-32S ESP-WROOM-32 WiFi | https://smile.amazon.com/DORHEA-Development-Microcontroller-NodeMCU-32S-ESP-WROOM-32/dp/B086MJGFVV/ |
| 1 | $4.45 | $4.45 | UBEC 3Amp: | https://smile.amazon.com/2-Pieces-Hobbywing-Switch-mode-UBEC-Helicopter-Quadcopter/dp/B01GHMW0C0 |
| 1 | $0.40 | $0.40 | SN74HCT125N quadruple bus buffer and voltage translator | https://www.digikey.com/product-detail/en/texas-instruments/SN74HCT125N/296-8386-5-ND/376860 |
| 1 | $0.10 | $0.10 | 10K Ohm resistor, 1/4 watt | https://www.digikey.com/en/products/detail/stackpole-electronics-inc/CF14JT10K0/1741265 |
| 1 | $0.10 | $0.10 | 0.1 uF electrolytic capacitor | https://www.digikey.com/en/products/detail/w%C3%BCrth-elektronik/860010672001/5728608 |
| 1 | $0.53 | $0.53 | 100 uF electrolytic capacitor | https://www.digikey.com/en/products/detail/panasonic-electronic-components/ECA-2AM101/245067 |
