# Water pump driver
This is SW for arduino nano for http://vektor-tools.com pump controller.
Arduino is a replacement and enhancement for the chip used in original solution.
The idea behind the project it is to be able to change delay which stops pump motor after turning water off. Original solution uses const time 10 seconds, which is too much for simple water installations. 
Arduino nano based version uses potentiometer to set time (0 - 10 seconds). The remaining functionality is exactly the same as in original ucontroller.
This version of the software is on a branch `original`

Currently on the master branch there is another enhancement.
Instead of spring based switch that starts pump motor when water pressure is dropped and check valve used for stopping then pump motor, a pressure sensor is used for detecting when hte water pressure drops (start pump motor) and when the water pressure is at a level that can be treated as turning water off (stop pump motor). To set max water pressure another potentiometer is used.
All pin used with a bit of description you can find in *.ino file.

You can find scheme for original pump controller on russian site:
https://forum.cxem.net/index.php?/blogs/entry/521-%D0%B2%D0%B5%D0%BA%D1%82%D0%BE%D1%80-%D0%B2%D0%BA%D0%B4-3%D1%80-%D1%80%D0%B5%D0%BB%D0%B5-%D0%B4%D0%B0%D0%B2%D0%BB%D0%B5%D0%BD%D0%B8%D1%8F-%D1%81-%D0%B7%D0%B0%D1%89%D0%B8%D1%82%D0%BE%D0%B9-%D0%BF%D0%BE-%D1%81%D1%83%D1%85%D0%BE%D0%BC%D1%83-%D1%85%D0%BE%D0%B4%D1%83/
