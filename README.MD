This project implements a solar tracker on a Raspberri Pi 4 computer and  some additional custom hardware.  

The Raspberri Pi recieves time information from a local realtime clock, the Adafruit PFC8523.  This RTC uses the I2C bus of the Pi to communicate time information.  Time info allows the Pi to accurately track sunrise, noon and sunset times for the local area.  

Briefly, a python scripts called solartracker.py is called at boot. It reads time information from a file called suntime.txt.  The information will consist of sunrise, sunset, and elapsed time for the daylight.  Solartracker.py will move to the full east position at dawn, then move the array incrementally throughout the day until it reaches the full west position at dusk.  It will calculate the amount the array should move during each small time interval to keep up with the sun.  There will be a light sensor that will determine the actual dawn (versus the predicted dawn from sunwait) and look for gross errors in time (such as a dead battery on the realtime clock).  At sunset, the array returns to the sull East position to wait for dawn. A small LCD screen will display important parameters and error messages.  Two manual buttons will allow the array to be manually positioned for maintenance or other reasons.   

A utility called sunwait is used in a crontab job to generate a daily file containing sunrise, sunset and total day length.  Sunwait is not documented here but is available on https://risacher.org/sunwait/ and https://github.com/risacher/sunwait.  Usage of sunwait is well documented on the github source, we will only document the commands needed for this project here.  At the time of this writing sunwait has a bug which involve an extra parenthesis on line 704.  We will attempt to fix this bug shortly. Currently sunwait is also not reporting times in local timezone format, it is reporting them in GMT but this may be an inconsistent bug.  We will fix this bug if we can figure it out. sunwait outputs to a fille called suntimes.txt, and overwrites the contents of the file each day.  A default version of this file is installed in order to make sure the program always runs properly first time.  A sampkle crontab file is included that starts sunwait.  Merge the contents of this crontab with the one you are using, if any.   

rc.local is a file that is run at boot, which called solartracker.  Merge the contents of the rc.local file in this project with the one on your pi located at /etc/rc.local.  Make sure there is an "&" character at the end of the line that calls solartracker.py. This will initiate the solartracker.py python script at the end of the boot process. 

solartracker2_4 ino.ino is a previous version of this program that ran on an Arduino.  It is brought in here for reference as to how these problems were solved previously.  It use a pair of photoresistors to sense the orintation of the solar array vs the sun as well as light/dark.  Settings were field calibrated and used analog inputs tot he arduino.  

Raspberry Pi I/O list: 

GPIO 0	SDA0 (I2C for real time clock)
GPIO 1  SCL0 (I2C for realtime clock)

GPIO 2 Motor forward manual button (use internal pullup resistor)
GPIO 3 Motor reverse manual button (use internal pullup resistor)

GPIO 14 TXD0  (Tx for LCD Screen serial interface)
GPIO 15 RXD0  (Rx for LCD Screen serial interface)
	





