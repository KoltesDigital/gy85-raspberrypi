# GY-85 on Raspberry Pi

A GY-85 IMU combines a ADXL345 (3-axis accelerometer), a HMC5883L (3-axis digital compass), and a ITG3205 (3-axis gyroscope).

The program displays the data coming from the three sensors. You may use it to determine the sensors' data ranges: as you'll see, data do not have the same amplitude and middle value, even between X Y Z of the same sensor.

## Build

	apt-get install libncurses5-dev
	scons
	./demo

## License

Copyright 2016 Bloutiouf

[MIT License](https://opensource.org/licenses/MIT)
