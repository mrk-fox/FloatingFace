<div align="center">
<h1>FloatingFace</h1><br>
  
A three-string-mounted small multifunctional cable roboter concept. <br>

![KiCad](https://img.shields.io/badge/kicad-%2300578F.svg?style=for-the-badge&logo=kicad&logoColor=white) 
![Espressif](https://img.shields.io/badge/espressif-E7352C.svg?style=for-the-badge&logo=espressif&logoColor=white)
![AutoDesk](https://img.shields.io/badge/autodesk-%23000000.svg?style=for-the-badge&logo=autodesk&logoColor=white)
[![License: CERN-OHL-S](https://img.shields.io/badge/License-CERN--OHL--S%20v2-red)](https://ohwr.org/cern_ohl_s_v2)
<br>

<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/full_fixed_render.bmp" alt="Flowers" style="width:auto;">
<div align="left">v1.0 aka "Floaty"</div>
</div>
<br>

## Key features

- 3-points mount
- cheeseplate with additional servo for mechanization
- automatic calibration
- custom driver PCB with three TMC2209 slots
- redundant visual and magnetic travel distance encoder
- AS5600 magnetic encoders
- MPU5060
- possibility for rope pulley if higher loads applied

I built this project as proof-of-concept system for a 3-point-mounted, portable cable roboter with an universal extensions interface.

## PCB

Floaty has a custom-designed driver developer board for three TMC2209 drivers. Look at the design! <br>
<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/delta_drive_v1.1_chem.png" alt="Flowers" style="width:auto;">
Thats all the layers overlapped...
<br><br><br>
<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/delta_drive_v1.1.jpg" alt="Flowers" style="width:auto;">
...and thats the front design. There is nothing on the back
<br><br><br>
<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/jlcpbc.png" alt="Flowers" style="width:auto;">

## Schematic
<br>
The overview schematic implementing a TCA9548 for the I2C controll of the three AS5600 sensors looks like this:
<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/overview_v1.1.png" alt="Flowers" style="width:auto;">

and that is the schematic for the delta_drive v1.1:
<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/delta_drive_v1.1_schematic.png" alt="Flowers" style="width:auto;">
https://github.com/mrk-fox/FloatingFace/blob/main/pictures/delta_drive_v1.1_schematic.png

## Math
The basics were worked out with simple linear algebra, which are explained in this video together with the calibration process:

[The math behind "Floaty", the cable roboter](https://youtu.be/A8Sq22G1rsY)

Further, to make the implementation easier, all the calculation were modelled in Simulink:
<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/simulink.jpg" alt="Flowers" style="width:auto;">

<br>

The movement algorithm itself works on command through these steps:
1. Validate coordinates
2. Calculate $\Delta(g_n)$ (individual motor run vectors)
3. Calculate time-based movement coefficients $c(\Delta(g_n))$
4. Move in accordance to coefficitens slowly until all motor sensors register rising edge on light sensor
5. Re-calculate path
6. Move fast using PCNT counters
7. Slow down 2 guide sensor revolutions before movement finish
8. Proceed until finish on magnetic encoders feedback
9. Calibrate angles on finish

For angle calibration, Floaty uses the MP5060 sensor which gives an angle relative to gravity and the direction of the orthogonal of the angled face of the sensor. With that data an iterative funciton is started ajusting one motor step at a time until an angle less than 10 degrees is reached. 

## Zine Page

<img src="https://github.com/mrk-fox/FloatingFace/blob/main/pictures/fallout_zine.png" alt="Flowers" style="width:auto;">

## Credit
I thank this project to... <br>
... [KiCad](https://www.kicad.org/)<br>
... [HackClub Fallout](https://fallout.hackclub.com/)<br>
... [Autodesk Inventor](https://www.autodesk.com/de/products/inventor/overview)<br>
... [BenjaminW](https://www.thingiverse.com/BenjaminW/designs) and [Serj Minin](https://grabcad.com/serj.minin-1) for the models

## License

This hardware project is licensed under the **CERN Open Hardware Licence v2 - Strongly Reciprocal (CERN-OHL-S)**.

See the [LICENSE](./LICENSE) file for details, or visit https://ohwr.org/cern_ohl_s_v2.




