# The project
In this project a method to measure the wind speed is proposed. To fulfill this task analog data coming from a 9-axis sensor is used. The device is  placed in an element that rotates according to the wind speed, in this case in an anemometer but it can also be place, for example, in a wind turbine.
In a first phase data is capture from the 9-axis sensors alongside with the true wind speed.
Once there are enough samples at different wind speeds, the algorithms are developed. In this process polynomial and lineal relationships are obtain with the data captured before.
Finally, the device is tested. It’s place in the rotatory surface of the anemometer and powered with batteries. Errors of less than 2m/s are obtained. The final purpose of this device is to act as a data logger. Via Bluethooth sends in real time the True Wind Speed.
## Code of the data capture process
This code is used [code](https://github.com/jsa-m/Wind_IOT/blob/main/WIND_IOT_ESP1_captura/WIND_IOT_ESP1_captura.ino) to capture the data of the accelerometer, gyroscope and the magnetometer in the three axis (x, y, z) all the date is sent in two characteristics. The data is converted from floating point to hex and sent via Bluetooth to a secondary esp-32 who acts as a data logger. Forty-nine characters are sent each second.  
The device that captures the true wind speed, receives compiles and sends the data by serial port for analysis with this  [code] (https://github.com/jsa-m/Wind_IOT/blob/main/WIND_IOT_ESP2_captura/WIND_IOT_ESP2_captura.ino)

## Code to estimate the true wind speed.
This [code](https://github.com/jsa-m/Wind_IOT/blob/main/WIND_IOT_ESP1_APP/WIND_IOT_ESP1_APP.ino) is used for the final application. The device works as a commercial environmental sensor, it sends every second by BLE notifications the wind speed data that can be consulted by any user within the range of the device.
