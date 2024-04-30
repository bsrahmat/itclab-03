# Python Testing

Python _Testing is a simple program to test the iTCLab Kit using Python. Before the Python program is run, of course, the appropriate Arduino program must have been embedded in the iTCLab Kit.
<br>
</br>

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-01/blob/main/itclab01a.jpg" alt="" class="img-responsive" width="700">
</p>

## About these iTCLab kits :

iTCLab - Internet-Based Temperature Control Lab. Temperature control kit for feedback control applications with an ESP32 Microcontroller, LED, two heaters, and two temperature sensors. The heating power output is adjusted to maintain the desired temperature setpoint. Heat energy from the heater is transferred by conduction, convection, and radiation to the temperature sensor. Heat is also transferred from the device to the environment.

### More about iTCLab:

- It is inspired by <a href="https://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl" target="_blank">The TCLab Product of Brigham Young University (BYU), one of the private campuses in Provo, Utah, United States of America.</a>
- Miniature Control System in a Pocket.
- Practical IoT Learning Package Tools.
- IoT programming Kits.
- IoT-Based Control System Practice Devices.
- It can be used to learn System Dynamics and Control Systems.
- It can be used to learn Arduino and Python programming.
- It can be used to learn AI and Machine Learning Programming.
- And others.

The fundamental difference between iTCLab and BYU's TCLab product is the replacement of the Arduino Uno microcontroller with the ESP32. By using the ESP32, iTCLab has the ability to connect to the Internet of Things (IoT).
<br>
</br>

## iTCLab Upper Temperature Limit Description:

The upper temperature limit of the iTCLab Kit is 60 degrees Celsius. Therefore, when experimenting with this Kit, this Upper Temperature Limit must not be exceeded. Violation of this provision could cause damage (burning) to the components.

Although the upper limit is 60 degrees Celsius, it is still sufficient for experimenting with this Kit. And it is sufficient to see the performance of a control method. For example, control using Proportional Integral and Derivative (PID). Or to see the effect of tuning the PID parameters using the Machine Learning method. An illustration of the capabilities of this iTCLab Kit can be seen from the illustration of the performance of the BYU TCLab, as seen in the following simulation.

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-01/blob/main/pid_control.gif" alt="" class="img-responsive" width="700">
</p>

## Coding Upper Temperature Limit

It is necessary to set a limit so that the iTCLab Kit always operates in a safe area. It must not exceed the upper limit of 60 degrees Celsius. The following is an example of an Arduino program script that must be added every time you experiment with this Kit. In the Loop, it is added that if it reaches the specified upper limit (it can be lowered slightly, for example 55 degrees Celsius), then the heater must be turned off.

```
void loop() {
  // put your main code here, to run repeatedly:
  cektemp();
  if (cel > upper_temperature_limit){
    Q1off();
    ledon();
  }
  else {
    Q1on();
    ledoff();
  }
  if (cel1 > upper_temperature_limit){
    Q2off();
    ledon();
  }
  else {
    Q2on();
    ledoff();
  }
  delay (100);
}

```
<br>
</br>

## Python Testing

Python _Testing is a simple program to test the iTCLab Kit using Python. Before the Python program is run, of course, the appropriate Arduino program must have been embedded in the iTCLab Kit.
<br>
</br>

## Required Equipment:

- <a href="https://shopee.co.id/product/78709625/11589970517/" target="_blank">iTCLab Kit</a>

- <a href="https://github.com/bsrahmat/itclab-03/blob/main/arduino_python.ino" target="_blank">arduino_python.ino Program</a>

- <a href="https://github.com/bsrahmat/itclab-03/blob/main/python_testing.ipynb" target="_blank">python_testing.ipynb Program</a>

- <a href="https://github.com/bsrahmat/itclab-03/blob/main/itclab.py" target="_blank">itclab.py Program</a>

- <a href="https://github.com/bsrahmat/itclab-03/blob/main/Python_Testing.pdf" target="_blank">Python_Testing.pdf Tutorial</a>


### Another alternative is to download the tutorial:

- <a href="https://www.academia.edu/116412601" target="_blank">https://www.academia.edu/116412601</a>

- <a href="https://www.researchgate.net/publication/379052435" target="_blank">https://www.researchgate.net/publication/379052435</a>

<br>
</br>

### Steps:
1.	Upload the arduino_python.ino program to the iTCLab Kit
2.	Put the itclab.py file in the same folder as the python_testing.ipynb program
3.	Run the python_testing.ipynb program via Jupyter Notebook
<br>
</br>

### Here is the arduino_python.ino coding script:

```
/*
  iTCLab Internet-Based Temperature Control Lab Firmware
  Jeffrey Kantor, Initial Version
  John Hedengren, Modified
  Oct 2017
  Basuki Rahmat, Modified
  April 2022

  This firmware is loaded into the Internet-Based Temperature Control Laboratory ESP32 to
  provide a high level interface to the Internet-Based Temperature Control Lab. The firmware
  scans the serial port looking for case-insensitive commands:

  Q1        set Heater 1, range 0 to 100% subject to limit (0-255 int)
  Q2        set Heater 2, range 0 to 100% subject to limit (0-255 int)
  T1        get Temperature T1, returns deg C as string
  T2        get Temperature T2, returns dec C as string
  VER       get firmware version string
  X         stop, enter sleep mode

  Limits on the heater can be configured with the constants below.
*/

#include <Arduino.h>

// constants
const String vers = "1.04";    // version of this firmware
const int baud = 115200;       // serial baud rate
const char sp = ' ';           // command separator
const char nl = '\n';          // command terminator

// pin numbers corresponding to signals on the iTCLab Shield
const int pinT1   = 34;         // T1
const int pinT2   = 35;         // T2
const int pinQ1   = 32;         // Q1
const int pinQ2   = 33;         // Q2
const int pinLED  = 26;         // LED

//Q1 32 - T1 34
//Q2 33 - T2 35

// setting PWM properties
const int freq = 5000; //5000
const int ledChannel = 0;
const int Q1Channel = 1;
const int Q2Channel = 2;
const int resolutionLedChannel = 8; //Resolution 8, 10, 12, 15
const int resolutionQ1Channel = 8; //Resolution 8, 10, 12, 15
const int resolutionQ2Channel = 8; //Resolution 8, 10, 12, 15

const double upper_temperature_limit = 59;

// global variables
char Buffer[64];               // buffer for parsing serial input
String cmd;                    // command 
double pv = 0;                 // pin value
float level;                   // LED level (0-100%)
double Q1 = 0;                 // value written to Q1 pin
double Q2 = 0;                 // value written to Q2 pin
int iwrite = 0;                // integer value for writing
float dwrite = 0;              // float value for writing
int n = 10;                    // number of samples for each temperature measurement

void parseSerial(void) {
  int ByteCount = Serial.readBytesUntil(nl,Buffer,sizeof(Buffer));
  String read_ = String(Buffer);
  memset(Buffer,0,sizeof(Buffer));
   
  // separate command from associated data
  int idx = read_.indexOf(sp);
  cmd = read_.substring(0,idx);
  cmd.trim();
  cmd.toUpperCase();

  // extract data. toInt() returns 0 on error
  String data = read_.substring(idx+1);
  data.trim();
  pv = data.toFloat();
}

// Q1_max = 100%
// Q2_max = 100%

void dispatchCommand(void) {
  if (cmd == "Q1") {   
    Q1 = max(0.0, min(25.0, pv));
    iwrite = int(Q1 * 2.0); // 10.? max
    iwrite = max(0, min(255, iwrite));    
    ledcWrite(Q1Channel,iwrite);
    Serial.println(Q1);
  }
  else if (cmd == "Q2") {
    Q2 = max(0.0, min(25.0, pv));
    iwrite = int(Q2 * 2.0); // 10.? max
    iwrite = max(0, min(255, iwrite));
    ledcWrite(Q2Channel,iwrite);   
    Serial.println(Q2);
  }
  else if (cmd == "T1") {
    float mV = 0.0;
    float degC = 0.0;
    for (int i = 0; i < n; i++) {
        mV = (float) analogRead(pinT1) * 0.322265625;
        degC = degC + mV/10.0;
    }
    degC = degC / float(n);   
    
    Serial.println(degC);
  }
  else if (cmd == "T2") {
    float mV = 0.0;
    float degC = 0.0;
    for (int i = 0; i < n; i++) {
         mV = (float) analogRead(pinT2) * 0.322265625;
         degC = degC + mV/10.0;
   }
    degC = degC / float(n);
    Serial.println(degC);
  }
  else if ((cmd == "V") or (cmd == "VER")) {
    Serial.println("TCLab Firmware Version " + vers);
  }
  else if (cmd == "LED") {
    level = max(0.0, min(100.0, pv));
    iwrite = int(level * 0.5);
    iwrite = max(0, min(50, iwrite));    
    ledcWrite(ledChannel, iwrite);      
    Serial.println(level);
  }  
  else if (cmd == "X") {
    ledcWrite(Q1Channel,0);
    ledcWrite(Q2Channel,0);
    Serial.println("Stop");
  }
}

// check temperature and shut-off heaters if above high limit
void checkTemp(void) {
    float mV = (float) analogRead(pinT1) * 0.322265625;
    //float degC = (mV - 500.0)/10.0;
    float degC = mV/10.0;
    if (degC >= upper_temperature_limit) {
      Q1 = 0.0;
      Q2 = 0.0;
      ledcWrite(Q1Channel,0);
      ledcWrite(Q2Channel,0);
      //Serial.println("High Temp 1 (> upper_temperature_limit): ");
      Serial.println(degC);
    }
    mV = (float) analogRead(pinT2) * 0.322265625;
    //degC = (mV - 500.0)/10.0;
    degC = mV/10.0;
    if (degC >= upper_temperature_limit) {
      Q1 = 0.0;
      Q2 = 0.0;
      ledcWrite(Q1Channel,0);
      ledcWrite(Q2Channel,0);
      //Serial.println("High Temp 2 (> upper_temperature_limit): ");
      Serial.println(degC);
    }
}

// arduino startup
void setup() {
  //analogReference(EXTERNAL);
  Serial.begin(baud); 
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  // configure pinQ1 PWM functionalitites
  ledcSetup(Q1Channel, freq, resolutionQ1Channel);
  
  // attach the channel to the pinQ1 to be controlled
  ledcAttachPin(pinQ1, Q1Channel); 

  // configure pinQ2 PWM functionalitites
  ledcSetup(Q2Channel, freq, resolutionQ2Channel);
  
  // attach the channel to the pinQ2 to be controlled
  ledcAttachPin(pinQ2, Q2Channel);   

  // configure pinLED PWM functionalitites
  ledcSetup(ledChannel, freq, resolutionLedChannel);
  
  // attach the channel to the pinLED to be controlled
  ledcAttachPin(pinLED, ledChannel); 

  ledcWrite(Q1Channel,0);
  ledcWrite(Q2Channel,0);
}

// arduino main event loop
void loop() {
  parseSerial();
  dispatchCommand();
  checkTemp();
}

```

<br>
</br>

### Here is the itclab.py coding script:

```
import sys
import time
import numpy as np
try:
    import serial
except:
    import pip
    pip.main(['install','pyserial'])
    import serial
from serial.tools import list_ports
        
class iTCLab(object):

    def __init__(self, port=None, baud=115200):
        port = self.findPort()
        print('Opening connection')
        self.sp = serial.Serial(port=port, baudrate=baud, timeout=2)
        self.sp.flushInput()
        self.sp.flushOutput()
        time.sleep(3)
        print('iTCLab connected via Arduino on port ' + port)
        
    def findPort(self):
        found = False
        for port in list(list_ports.comports()):
            # Arduino Uno
            if port[2].startswith('USB VID:PID=16D0:0613'):
                port = port[0]
                found = True
            # Arduino HDuino
            if port[2].startswith('USB VID:PID=1A86:7523'):
                port = port[0]
                found = True                
            # Arduino Leonardo
            if port[2].startswith('USB VID:PID=2341:8036'):
                port = port[0]
                found = True
            # Arduino ESP32
            if port[2].startswith('USB VID:PID=10C4:EA60'):
                port = port[0]
                found = True
            # Arduino ESP32 - Tipe yg berbeda
            if port[2].startswith('USB VID:PID=1A86:55D4'):
                port = port[0]
                found = True
        if (not found):
            print('Arduino COM port not found')
            print('Please ensure that the USB cable is connected')
            print('--- Printing Serial Ports ---')            
            for port in list(serial.tools.list_ports.comports()):
                print(port[0] + ' ' + port[1] + ' ' + port[2])
            print('For Windows:')
            print('  Open device manager, select "Ports (COM & LPT)"')
            print('  Look for COM port of Arduino such as COM4')
            print('For MacOS:')
            print('  Open terminal and type: ls /dev/*.')
            print('  Search for /dev/tty.usbmodem* or /dev/tty.usbserial*. The port number is *.')
            print('For Linux')
            print('  Open terminal and type: ls /dev/tty*')
            print('  Search for /dev/ttyUSB* or /dev/ttyACM*. The port number is *.')
            print('')
            port = input('Input port: ')
            # or hard-code it here
            #port = 'COM3' # for Windows
            #port = '/dev/tty.wchusbserial1410' # for MacOS
        return port
    
    def stop(self):
        return self.read('X')
    
    def version(self):
        return self.read('VER')
    
    @property
    def T1(self):
        self._T1 = float(self.read('T1'))
        return self._T1
    
    @property
    def T2(self):
        self._T2 = float(self.read('T2'))
        return self._T2
        
    def LED(self,pwm):
        pwm = max(0.0,min(100.0,pwm))/2.0
        self.write('LED',pwm)
        return pwm

    def Q1(self,pwm):
        pwm = max(0.0,min(100.0,pwm)) 
        self.write('Q1',pwm)
        return pwm
        
    def Q2(self,pwm):
        pwm = max(0.0,min(100.0,pwm)) 
        self.write('Q2',pwm)
        return pwm

    # save txt file with data and set point
    # t = time
    # u1,u2 = heaters
    # y1,y2 = tempeatures
    # sp1,sp2 = setpoints
    def save_txt(self,t,u1,u2,y1,y2,sp1,sp2):
        data = np.vstack((t,u1,u2,y1,y2,sp1,sp2))  # vertical stack
        data = data.T                 # transpose data
        top = 'Time (sec), Heater 1 (%), Heater 2 (%), ' \
          + 'Temperature 1 (degC), Temperature 2 (degC), ' \
          + 'Set Point 1 (degC), Set Point 2 (degC)' 
        np.savetxt('data.txt',data,delimiter=',',header=top,comments='')

    def read(self,cmd):
        cmd_str = self.build_cmd_str(cmd,'')
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
        except Exception:
            return None
        return self.sp.readline().decode('UTF-8').replace("\r\n", "")
    
    def write(self,cmd,pwm):       
        cmd_str = self.build_cmd_str(cmd,(pwm,))
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
        except:
            return None
        return self.sp.readline().decode('UTF-8').replace("\r\n", "")
    
    def build_cmd_str(self,cmd, args=None):
        """
        Build a command string that can be sent to the arduino.
    
        Input:
            cmd (str): the command to send to the arduino, must not
                contain a % character
            args (iterable): the arguments to send to the command
        """
        if args:
            args = ' '.join(map(str, args))
        else:
            args = ''
        return "{cmd} {args}\n".format(cmd=cmd, args=args)
        
    def close(self):
        try:
            self.sp.close()
            print('Arduino disconnected successfully')
        except:
            print('Problems disconnecting from Arduino.')
            print('Please unplug and reconnect Arduino.')
        return True   
```

<br>
</br>


### Here is the python_testing.ipynb coding script:

```
import itclab
import time
# Connect to Arduino
a = itclab.iTCLab()
print('LED On')
a.LED(100)
# Pause for 1 second
time.sleep(1.0)
print('LED Off')
a.LED(0)
a.close()

```

```
import itclab
import time

# Connect to Arduino
a = itclab.iTCLab()

# Get Version
print(a.version)

# Turn LED on
print('LED On')
a.LED(100)

# Taper LED off
for i in range(100,-1,-10):
    print('LED Power ' + str(i))
    time.sleep(0.5)
    a.LED(i)

a.close()
```

<br>
</br>


Please run it in the Jupyter notebook environment, with the iTCLab Kit connected to a PC or Laptop (with the arduino_python.ino program already embedded in it). Then the results should be as shown in the following Figures.

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-03/blob/main/python_test1.jpg" alt="" class="img-responsive" width="700">
</p>

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-03/blob/main/python_test2.jpg" alt="" class="img-responsive" width="700">
</p>

<br>
</br>

## An example of a publication related to this Kits:

| Num  | iTCLab Publication Articles                                                                                                                                                     | Download BibTex Citation                                                                 |
| -----|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------|
|  01  |<a href="https://www.researchgate.net/publication/368801589">iTCLab Temperature Monitoring and Control System Based on PID and Internet of Things (IoT)</a>                      | [Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_iTCLab01.bib) |
|  02  |<a href="https://www.researchgate.net/publication/378937580">Temperature Monitoring via the Internet of Things Using PID-iTCLab</a>                                              | [Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_iTCLab02.bib) |
|  03  |<a href="https://www.researchgate.net/publication/378938102">On/Off Temperature Monitoring and Control via the Internet of Things Using iTCLab Kit</a>                           | [Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_iTCLab03.bib) |
|  04  |<a href="https://ieeexplore.ieee.org/document/10420130">ITCLab PID Control Tuning Using Deep Learning</a>                                                                        | [Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas01.bib)    |
<br>
</br>

## Scopus Publications:

| Num  | Scopus Publication Articles                                                                                                                                                      | Download BibTex Citation                                                                    |
| -----|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------|
|  01  |<a href="https://www.researchgate.net/publication/312337382">VLP Image Segmentation System Using Cellular Neural Network Optimized by Adaptive fuzzy & NFA</a>                    | [Rahmat et al. (2016)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas10.bib)       |
|  02  |<a href="https://www.researchgate.net/publication/313218211">Designing Intelligent Fishcarelab System (IFS) as Modern Koi Fish Farming System</a>                                 | [Rahmat et al. (2017)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas09.bib)       |
|  03  |<a href="https://www.researchgate.net/publication/332119322">An Improved Mean Shift Performance Using Switching Kernels for Indonesia VLP Tracking Video</a>                      | [Rahmat et al. (2018)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas08.bib)       |
|  04  |<a href="https://www.researchgate.net/publication/327382646">An Improved Mean Shift Using Adaptive Fuzzy Gaussian Kernel for Indonesia VLP Tracking</a>                           | [Rahmat et al. (2018)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas07.bib)       |
|  05  |<a href="https://www.researchgate.net/publication/349517987">Comparison of B-Value Predictions as Earthquake Precursors using ELM and Deep Learning</a>                           | [Rahmat et al. (2020)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas06.bib)       |
|  06  |<a href="https://www.researchgate.net/publication/343186219">Video-based Tancho Koi Fish Tracking System Using CSK, DFT, and LOT</a>                                              | [Rahmat et al. (2020)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas05.bib)       |
|  07  |<a href="https://ieeexplore.ieee.org/document/9321094">Combining of extraction butterfly image using color, texture and form features</a>                                         | [Dhian,Rahmat et al. (2020)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas17.bib) |
|  08  |<a href="https://www.researchgate.net/publication/350296152">Video-Based Container Tracking System Using Deep Learning</a>                                                        | [Rahmat et al. (2021)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas04.bib)       |
|  09  |<a href="https://www.researchgate.net/publication/356806533">ELM-Based Indonesia Vehicle License Plate Recognition System</a>                                                     | [Rahmat et al. (2021)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas03.bib)       |
|  10  |<a href="https://ieeexplore.ieee.org/document/10010310">Indonesia's Open Unemployment Rate Prediction System Using Deep Learning</a>                                              | [Rahmat et al. (2022)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas02.bib)       |
|  11  |<a href="https://www.engineeringletters.com/issues_v30/issue_3/EL_30_3_16.pdf">Performance of Optimized CSK, DFT, and LOT for Video-based Container Tracking System using SA</a>  | [Endra,Rahmat et al. (2022)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas16.bib) |
|  12  |<a href="https://ieeexplore.ieee.org/document/10010285">Poor Population Classification System Using Convolutional Neural Network</a>                                              | [Suaib,Rahmat et al. (2022)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas15.bib) |
|  13  |<a href="https://internetworkingindonesia.org/Issues/Vol14-No2-2022/iij_vol14_no2_2022_halim.pdf">Poverty Prediction System on Indonesian Population Using Deep Learning</a>      | [Suaib,Rahmat et al. (2022)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas14.bib) |
|  14  |<a href="https://ieeexplore.ieee.org/document/10420130">ITCLab PID Control Tuning Using Deep Learning</a>                                                                         | [Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas01.bib)       |
|  15  |<a href="https://ieeexplore.ieee.org/document/10420364/">CFD Simulation of the Effect of Adding a Sludge Zone to the Sequencing Batch Reactor for Fluid Turbulence</a>            | [Novi, Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas13.bib) |
|  16  |<a href="https://ieeexplore.ieee.org/document/10420129/">Detect Areas of Upward and Downward Fluctuations in Bitcoin Prices Using Patterned Datasets</a>                          | [Rizky,Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas12.bib) |
|  17  |<a href="https://ieeexplore.ieee.org/document/10419873">The Impact of Message Replication on VDTN Spray Protocol for Smart City</a>                                               | [Agus, Rahmat et al. (2023)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas11.bib) |
|  18  |<a href="https://joiv.org/index.php/joiv/article/view/1543">Minimum, Maximum, and Average Implementation of Patterned Datasets in Mapping Cryptocurrency Fluctuation Patterns</a> | [Rizky,Rahmat et al. (2024)](https://github.com/bsrahmat/itclab-01/blob/main/bib_bas18.bib) |

<br>
</br>

## Books Publications:

| Num  | Books Publication Articles                                                                                                                              | Download BibTex Citation                                                                        |
| -----|---------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
|  01  |<a href="https://github.com/bsrahmat/ebook-01">Fuzzy and Artificial Neural Networks Programming for Intelligent Control Systems (Indonesian version)</a> | [Rahmat et al. (2019)](https://github.com/bsrahmat/ebook-01/blob/main/EBook01_Fuzzy_JST.bib)    |
|  02  |<a href="https://github.com/bsrahmat/ebook-02">Intelligent Robot Programming Using Arduino (Indonesian version)</a>                                      | [Rahmat et al. (2020)](https://github.com/bsrahmat/ebook-02/blob/main/EBook02_Kinect_Robot.bib) |
|  03  |<a href="https://github.com/bsrahmat/ebook-03">Deep Learning Programming Using Python (Indonesian version)</a>                                           | [Rahmat et al. (2021)](https://github.com/bsrahmat/ebook-03/blob/main/EBook03_DL.bib)           |
|  04  |<a href="https://github.com/bsrahmat/ebook-04">MQTT Brokers as Supporting IoT Services (Indonesian version)</a>                                          | [Rahmat et al. (2022)](https://github.com/bsrahmat/ebook-04/blob/main/EBook04_MQTT_Broker.bib)  |
|  05  |<a href="https://github.com/bsrahmat/ebook-05">Integrated Web-Based E-Finance System (Indonesian version)</a>                                            | [Rahmat et al. (2023)](https://github.com/bsrahmat/ebook-05/blob/main/EBook05_EFinance.bib)     |
|  06  |<a href="https://github.com/bsrahmat/ebook-06">Asset Management Information System (Indonesian version)</a>                                              | [Rahmat et al. (2023)](https://github.com/bsrahmat/ebook-06/blob/main/EBook06_Simaset.bib)      |
|  07  |<a href="https://github.com/bsrahmat/ebook-07">Village Management Information System (Indonesian version)</a>                                            | [Rahmat et al. (2023)](https://github.com/bsrahmat/ebook-07/blob/main/EBook07_Simdes.bib)       |
|  08  |<a href="https://github.com/bsrahmat/ebook-08">Retribution Management Information System (Indonesian version)</a>                                        | [Rahmat et al. (2023)](https://github.com/bsrahmat/ebook-08/blob/main/EBook08_Simretribusi.bib) |
|  09  |<a href="https://github.com/bsrahmat/ebook-09">Gender Management Information System (Indonesian version)</a>                                             | [Rahmat et al. (2023)](https://github.com/bsrahmat/ebook-09/blob/main/EBook09_Sagada.bib)       |

<br>
</br>

## Other publications by the researcher:

https://fitri.academia.edu/BasukiRahmat

https://www.researchgate.net/profile/Basuki-Rahmat-2

https://scholar.google.com/citations?user=BjCi4AgAAAAJ&hl=en

<br>
</br>

## Buy the iTCLab Kits at Shopee:

<p align="center">
<a href="https://shopee.co.id/product/78709625/11589970517" target="_blank"><img src="https://github.com/bsrahmat/itclab-01/blob/main/shopee_kit1.jpg" alt="" class="img-responsive" width="700">
</a>
</p>
<br>
</br>

## Other research by the IO-T.Net research team can be found at:

https://www.io-t.net/

<p align="center">
<a href="https://www.io-t.net/" target="_blank"><img src="https://github.com/bsrahmat/robot-bnu/blob/main/iot.png" alt="" class="img-responsive" width="500">
</a>
</p>

<br>
</br>

## Other research by the I-OT.Net research team can be found also at:

https://www.i-ot.net/

<p align="center">
<a href="https://www.i-ot.net/" target="_blank"><img src="https://github.com/bsrahmat/fuzzy-neural/blob/main/iot_logo.png" alt="" class="img-responsive" width="500">
</a>
</p>

