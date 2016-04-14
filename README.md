# WS-IOT-Home-Arduino
Autobahn Web socket IOT Home for Arduino Yun

Code includes JS, Arduino C, Python

Descriptions:

1. The websocket server is set on the Aruino Yun with python code.
2. The Python code setup the client application and server at once which is a standalone server.
3. The JavaScript connect to this server and use remote process calls to send the serial message to MCU 32u4.
4. The Python code bypass the Yun bridge to communicate with CPU, Linino AR 9331.
5. Arduino MCU side can send sensor data to the website (Temperature, humidity, etc)
6. Arduino MCU side can receive control instructions from the website

Below is setup guide provided by Tobias Oberstein.

http://tavendo.com/blog/post/arduino-yun-with-autobahn/

Note : The autobahn version should be 0.9.1 or the standalone server will fail.

Note : The bridge function will be bypassed after commenting the last line of /etc/inittab

Note : The python will autostart after reboot after editing /etc/rc.local

Applications:
All the features below can be controled by a responsive web on the Phone / tablet / PC.

1. Turn on the air conditioner
2. Turn on the led light
3. Use Buzzer to make sounds
4. Collaberate with Pixy
5. Collaberate with Pixy Pan/tilt
6. Real time Smoothie Chart

Any question or advice, please mail me.

tim9510019@hotmail.com
