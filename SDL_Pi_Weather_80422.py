#
#
#
#  SDL__Pi_Weather_80422.py - Raspberry Pi Python Library for SwitchDoc Labs WeatherRack.
#
#  SparkFun Weather Station Meters
#  Argent Data Systems
#  Created by SwitchDoc Labs February 13, 2015 
#  Released into the public domain.
#    Version 1.3 - remove 300ms Bounce
#    

# imports

import sys
import time as time_
import Adafruit_ADS1x15
import RPi.GPIO as GPIO
from datetime import *

GPIO.setwarnings(False)


# constants

SDL_MODE_INTERNAL_AD = 0
SDL_MODE_I2C_ADS1015 = 1

#sample mode means return immediately.  THe wind speed is averaged at sampleTime or when you ask, whichever is longer
SDL_MODE_SAMPLE = 0
#Delay mode means to wait for sampleTime and the average after that time.
SDL_MODE_DELAY = 1

WIND_FACTOR = 2.400 

# Helper Functions


def fuzzyCompare(compareValue, value):
	
	VARYVALUE = 0.05

   	if ( (value > (compareValue * (1.0-VARYVALUE)))  and (value < (compareValue *(1.0+VARYVALUE))) ):
     		return True
        
	return False
 
def voltageToDegrees(value, defaultWindDirection):
    	# Note:  The original documentation for the wind vane says 16 positions.  Typically only recieve 8 positions.  And 315 degrees was wrong.
     
    	# For 5V, use 1.0.  For 3.3V use 0.66
	ADJUST3OR5 = 0.66
	PowerVoltage = 3.3


	if (fuzzyCompare(3.84 * ADJUST3OR5, value)):
	        return 0.0

	if (fuzzyCompare(1.98 * ADJUST3OR5, value)):
	        return 22.5

	if (fuzzyCompare(2.25 * ADJUST3OR5, value)):
		return 45
				  
	if (fuzzyCompare(0.41 * ADJUST3OR5, value)):
		return 67.5
					  
	if (fuzzyCompare(0.45 * ADJUST3OR5, value)):
		return 90.0

	if (fuzzyCompare(0.32 * ADJUST3OR5, value)):
		return 112.5

	if (fuzzyCompare(0.90 * ADJUST3OR5, value)):
		return 135.0

	if (fuzzyCompare(0.62 * ADJUST3OR5, value)):
		return 157.5

	if (fuzzyCompare(1.40 * ADJUST3OR5, value)):
		return 180

	if (fuzzyCompare(1.19 * ADJUST3OR5, value)):
		return 202.5

	if (fuzzyCompare(3.08 * ADJUST3OR5, value)):
		return 225

	if (fuzzyCompare(2.93 * ADJUST3OR5, value)):
		return 247.5

	if (fuzzyCompare(4.62 * ADJUST3OR5, value)):
		return 270.0
	
	if (fuzzyCompare(4.04 * ADJUST3OR5, value)):
		return 292.5

	if (fuzzyCompare(4.34 * ADJUST3OR5, value)): # chart in manufacturers documentation wrong
		return 315.0
	
	if (fuzzyCompare(3.43 * ADJUST3OR5, value)):
		return 337.5
	
	return defaultWindDirection  # return previous value if not found

# return current microseconds
def micros():

	microseconds = int(round(time_.time() * 1000000))
	return microseconds




class SDL_Pi_Weather_80422:


	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)


	# instance variables
	_currentWindCount = 0
	_currentRainCount = 0
	_shortestWindTime = 0

 	_pinAnem = 0
   	_pinRain = 0
     	_intAnem = 0
      	_intRain = 0
       	_ADChannel = 0
	_ADMode = 0

        _currentRainCount = 0
        _currentWindCount = 0
        _currentWindSpeed = 0.0
 	_currentWindDirection = 0.0

        _lastWindTime = 0
	_shortestWindTime = 0
		         
	_sampleTime = 5.0
	_selectedMode = SDL_MODE_SAMPLE
 	_startSampleTime = 0

	_currentRainMin = 0
	_lastRainTime = 0

	_ads1015 = 0

	def __init__(self, pinAnem, pinRain, intAnem, intRain, ADMode ):


		GPIO.setup(pinAnem, GPIO.IN)
		GPIO.setup(pinRain, GPIO.IN)
		
		# when a falling edge is detected on port pinAnem, regardless of whatever   
		# else is happening in the program, the function callback will be run  
	
		GPIO.add_event_detect(pinAnem, GPIO.RISING, callback=self.serviceInterruptAnem )  
		GPIO.add_event_detect(pinRain, GPIO.RISING, callback=self.serviceInterruptRain )  

		ADS1015 = 0x00  # 12-bit ADC
		# Select the gain
		self.gain = 2/3  # +/- 6.144V (2/3)
		#self.gain = 1  # +/- 4.096V (1)

		# Select the sample rate
		self.sps = 250  # 250 samples per second

		# Initialise the ADC using the default mode (use default I2C address)
		# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
		self.ads1015 = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)     # Changed default address (0x48) to 0x49

		SDL_Pi_Weather_80422._ADMode = ADMode


	# Wind Direction Routines

	def current_wind_direction(self):
    
    		if (SDL_Pi_Weather_80422._ADMode == SDL_MODE_I2C_ADS1015):
			value = self.ads1015.read_adc(1, self.gain, self.sps) # AIN1 wired to wind vane on WeatherPiArduino

      			voltageValue = value/1000

    		else:
      			# user internal A/D converter
      			voltageValue = 0.0
  
		direction = voltageToDegrees(voltageValue, SDL_Pi_Weather_80422._currentWindDirection)
    		return direction;



	def current_wind_direction_voltage(self):
    
    		if (SDL_Pi_Weather_80422._ADMode == SDL_MODE_I2C_ADS1015):
			value = self.ads1015.read_adc(1, self.gain, self.sps) # AIN1 wired to wind vane on WeatherPiArduino

      			voltageValue = value/1000

    		else:
      			# user internal A/D converter
      			voltageValue = 0.0
  
    		return voltageValue

	# Utility methods

	def reset_rain_total(self):
		SDL_Pi_Weather_80422._currentRainCount = 0;

	def accessInternalCurrentWindDirection(self):
   		return SDL_Pi_Weather_80422._currentWindDirection;

	def reset_wind_gust(self):
   		SDL_Pi_Weather_80422._shortestWindTime = 0xffffffff;


	def startWindSample(self, sampleTime):
  
      		SDL_Pi_Weather_80422._startSampleTime = micros();
     
      		SDL_Pi_Weather_80422._sampleTime = sampleTime;
      
	# get current wind 
	def get_current_wind_speed_when_sampling(self):


   		compareValue = SDL_Pi_Weather_80422._sampleTime*1000000;
 
   		if (micros() - SDL_Pi_Weather_80422._startSampleTime >= compareValue):
      			# sample time exceeded, calculate currentWindSpeed
      			timeSpan = (micros() - SDL_Pi_Weather_80422._startSampleTime);
 
      			SDL_Pi_Weather_80422._currentWindSpeed = (float(SDL_Pi_Weather_80422._currentWindCount)/float(timeSpan)) * WIND_FACTOR*1000000.0

			#print "SDL_CWS = %f, SDL_Pi_Weather_80422._shortestWindTime = %i, CWCount=%i TPS=%f" % (SDL_Pi_Weather_80422._currentWindSpeed,SDL_Pi_Weather_80422._shortestWindTime, SDL_Pi_Weather_80422._currentWindCount, float(SDL_Pi_Weather_80422._currentWindCount)/float(SDL_Pi_Weather_80422._sampleTime)) 

      			SDL_Pi_Weather_80422._currentWindCount = 0
      
      			SDL_Pi_Weather_80422._startSampleTime = micros()

 		#print "SDL_Pi_Weather_80422._currentWindSpeed=", SDL_Pi_Weather_80422._currentWindSpeed 
   		return SDL_Pi_Weather_80422._currentWindSpeed



	def setWindMode(self, selectedMode, sampleTime): # time in seconds 
  
  		SDL_Pi_Weather_80422._sampleTime = sampleTime;
  		SDL_Pi_Weather_80422._selectedMode = selectedMode;
  
  		if (SDL_Pi_Weather_80422._selectedMode == SDL_MODE_SAMPLE):
  			self.startWindSample(SDL_Pi_Weather_80422._sampleTime);
  
	#def get current values

	def get_current_rain_total(self):
        	rain_amount = 0.2794 * float(SDL_Pi_Weather_80422._currentRainCount)
        	SDL_Pi_Weather_80422._currentRainCount = 0;
		return rain_amount;




	def current_wind_speed(self): # in milliseconds
  
  		if (SDL_Pi_Weather_80422._selectedMode == SDL_MODE_SAMPLE):
    			SDL_Pi_Weather_80422._currentWindSpeed = self.get_current_wind_speed_when_sampling();
  		else:
    			# km/h * 1000 msec
    
    			SDL_Pi_Weather_80422._currentWindCount = 0;
    			delay(SDL_Pi_Weather_80422._sampleTime*1000);
    			SDL_Pi_Weather_80422._currentWindSpeed = (float(SDL_Pi_Weather_80422._currentWindCount)/float(SDL_Pi_Weather_80422._sampleTime)) * WIND_FACTOR;


  		return SDL_Pi_Weather_80422._currentWindSpeed;

	def get_wind_gust(self):
   
  		latestTime =SDL_Pi_Weather_80422._shortestWindTime;
  		SDL_Pi_Weather_80422._shortestWindTime=0xffffffff;
  		time=latestTime/1000000.0;  # in microseconds
		if (time == 0):
			return 0
		else:
  			return (1.0/float(time))*WIND_FACTOR;


  

	# Interrupt Routines


	def serviceInterruptAnem(self,channel):

	        #print "Anem Interrupt Service Routine"

  		currentTime= (micros()-SDL_Pi_Weather_80422._lastWindTime);

  		SDL_Pi_Weather_80422._lastWindTime=micros();

  		if(currentTime>1000):   # debounce
     			SDL_Pi_Weather_80422._currentWindCount = SDL_Pi_Weather_80422._currentWindCount+1

     			if(currentTime<SDL_Pi_Weather_80422._shortestWindTime):
     				SDL_Pi_Weather_80422._shortestWindTime=currentTime;
 



	def serviceInterruptRain(self,channel):
		
		#print "Rain Interrupt Service Routine"

  		currentTime=(micros()-SDL_Pi_Weather_80422._lastRainTime);

  		SDL_Pi_Weather_80422._lastRainTime=micros();
  		if(currentTime>500):   # debounce
       			SDL_Pi_Weather_80422._currentRainCount = SDL_Pi_Weather_80422._currentRainCount+1
    			if(currentTime<SDL_Pi_Weather_80422._currentRainMin):
     				SDL_Pi_Weather_80422._currentRainMin=currentTime;

