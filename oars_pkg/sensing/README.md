# Sensors

Scripts to collect and interpret sensor data and publish it to ROS go here.

`/raspi` contains a script for publishing orientation obtained from the IMU.
`ublox_gps.py` obtains location from the GPS and publishes it.
`map_class.py` and `wind.py` are template files, which could give structure to code dealing with localization or wind, respectively. If you're working on these things, you should figure out what structure you need, *and then* look at those files to see if they are useful to you.
