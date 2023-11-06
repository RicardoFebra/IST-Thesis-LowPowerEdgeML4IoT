# Sensors

This folder contains the libraries used to configure and collect data from the accelerometers used in the monitoring of the machines. The i2c_handler and spi_handler libraries are used to configure the i2c and spi protocols, respectively, and the accelerometers libraries make use of these libraries to handle the sensors.

The accelerometers libraries were created by analyzing the respective datasheets and are not explicit, as the names of the methods are respective to the accelerometer register that is being configured. So is advised to check the datasheet of the accelerometers to understand what each method does.