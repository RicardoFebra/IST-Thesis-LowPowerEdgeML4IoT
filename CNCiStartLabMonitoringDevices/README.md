# CNC iStartLab Monitoring Devices

Two devices were developed to monitor the raw vibration of a CNC machine from iStartLab. These devices were developed with two different boards, the SparkFun OpenLog Artemis and the Teensy 4.1. The collected data from both devices was stored in a microSD card and then processed in Python to extract features and train machine learning models.

The main reason for the development of two devices is that the LSM6DSO accelerometer was not available at the beginning of this monitoring use case. So the Openlog Artemis was used while the LSM6DSO was not available.

<p float="left">
  <img src="../Figures/CNCiStartLab/ArtemisLogger.png" height="250" />
  <img src="../Figures/CNCiStartLab/Teensy41.png" height="250" />
</p>

The image above shows the two devices developed. The OpenLog Artemis is on the left and the Teensy 4.1 is on the right.

## Teensy 4.1 Flowchart

![Devices_flowchart](../Figures/CNCiStartLab/DevicesFlowchart.png)

In this code, the DataHandler.h has the configuration of all the accelerometers used and has the initialization of all signal processing vectors and variables.

In some implementations, there was no need for the complete signal processing task (FFT, Statistical features, etc). So there are some commented lines in the code that can be used to implement the complete signal processing task. These are in the beginning of the DataHandler.h file in the "Define the desired tasks" section.