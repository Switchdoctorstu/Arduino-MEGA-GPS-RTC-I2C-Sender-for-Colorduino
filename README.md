# Arduino-MEGA-GPS-RTC-I2C-Sender-for-Colorduino


A project to build a household clock using a GPS reciever as the time source.

The display consists of a TM1683 based LED display along with I2C communications to drive colorduino display modules.
See my other projects for the source for the display modules.

I used a mega to give me enough serial ports to link in the gps and still have usb available. 


Arduino MEGA reads GPS, sets RTC, sends via I2C to Colorduino.

 Stuarts code to drive ublox GPS DS1307 RTC and TM1638 Display

    Reads the RTC 

    Displays time on TM1638
    Handles reset of RTC
    Handles button updates to RTC
    Added debug mode to clear serial noise
    Added gps handler
    Added I2C display module handler
    Added local BMP180 module
    Added watchdog
    Logging bmp and temperature
    Background graphic from log


 RF24 setup ok

*known bugs:*

GMT is sort of working but crudely adds an hour to the current hour and takes no account of date change points but this is only a problem when running on the rtc else the gps provides the correct date.

      Boolean isbst(int day, int month, int dow){
        if (month < 3 || month > 10)  return false; 
        if (month > 3 && month < 10)  return true; 
        int previousSunday = day - dow;
        if (month == 3) return previousSunday >= 25;
        if (month == 10) return previousSunday < 25;
        return false; // this line never gonna happen
    }


