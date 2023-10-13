# automowerduino
Arduino program for controlling Husqvarna autmower remotely

This program allows you to control your Husqvarna Automower remotely. This is tested on Automower 220 AC, not sure which other models are supported (let me know!).
Created and tested for Arduino MKR1000.

The software is two-fold:
- It runs a server accepting commands (eg home, auto, timer on/off,..)
- It sends out status updates

The status updates are formatted for a Loxone homeautomation server but could easily be adapted.

The wiring instructions are in the code. Tip: make sure to securily strap the arduino so it can't damage your mobo.

#### Request for changes
If you want to see any changes to the code, please feel free to **create a pull request**.

#### Disclaimer
This is working for over a year here, but I don't offer any guarantee. If you damage your mower modding it, you are reponsible. 