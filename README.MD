This project implements a solar tracker on a Raspberri Pi 4 computer and  some additional custom hardware.  

The Raspberri Pi recieves time information from a local realtime clock, the Adafruit PFC8523.  This RTC uses the I2C bus of the Pi to communicate time information.  Time info allows the Pi to accurately track sunrise, noon and sunset times for the local area.  

A utility called sunwait is used in a crontab job to generate a daily file containing sunrise, sunset and total day length.  Sunwait is not documented here but is available on https://risacher.org/sunwait/ and https://github.com/risacher/sunwait.  Usage of sunwait is well documented on the github source, we will only document the commands needed for this project here.  At the time of this writing sunwait has a bug which involve an extra parenthesis on line 704.  We will attempt to fix this bug shortly.  


