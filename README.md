# Arduino-MEGA-GPS-RTC-I2C-Sender-for-Colorduino
Arduino MEGA reads GPS, sets RTC, sends via I2C to Colorduino

// Stuarts code to drive u lox GPS DS1307 RTC and TM1638 Display

    Reads the RTC 

    Displays time on TM1638
    Handles reset of RTC
    Handles button updates to RTC
    Added debug mode to clear serial noise
    Added gps handler
    Added I2C display module handler
    Added local BMP180 module
    Added watchdog

Currently working on nrf24 connection for remote sensors
 RF24 setup ok

Need GMT
      Boolean isbst(int day, int month, int dow){
        if (month < 3 || month > 10)  return false; 
        if (month > 3 && month < 10)  return true; 
        int previousSunday = day - dow;
        if (month == 3) return previousSunday >= 25;
        if (month == 10) return previousSunday < 25;
        return false; // this line never gonna happen
    }


