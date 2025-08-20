## Software Setup <a id="SoftwareSetup"></a>

Before using the robot, software should be configured first, you will need to install the following (Programs have been tested and configured with both Windows 10 and Windows 11 operating systems. In case of using another operating system, we strongly recommend using any available searching engine to search for the installation for the software by yourself).
We'll be using Raspberry Pi to run image processing code written in python programming language. Commands will be sent via Serial communication to arduino wich will control the motors.

### Raspberry Pi Preparation and Configuration
In this section, we will go through all the steps to prepare and configure your Raspberry Pi. After completing the steps, you will be able to access your Raspberry Pi remotely and control it.
#### Raspberry Pi Operating System 
There are a lot of OS systems for Raspberry Pi with different versions. You can choose any of them from [official site](https://www.raspberrypi.com/software/operating-systems/), but it is preferable to install the same version we installed since it is tested and it works fine ([download link](https://downloads.raspberrypi.com/raspios_full_arm64/images/raspios_full_arm64-2023-02-22/2023-02-21-raspios-bullseye-arm64-full.img.xz)).
#### Raspberry Pi Imager installation
You will be setting up Raspberry Pi OS on your Raspberry Pi using the Raspberry Pi Imager tool, it will allow you to flash the image onto a USB flash drive. The provided steps cover the installation process, initial configuration, and essential steps to get your Raspberry Pi up and running. 
Using the Raspberry Pi Imager tool to flash the Raspberry Pi OS image onto a USB flash drive offers a convenient and efficient way to set up your Raspberry Pi. The Imager simplifies the installation process by providing a user-friendly interface to select the operating system and target storage medium. With just a few clicks, you can download the official Raspberry Pi OS image and write it directly to the USB flash drive. This eliminates the need for manual installation and configuration, saving you time and effort.
- To install Raspberry Pi Imager, Visit the official Raspberry Pi website at https://www.raspberrypi.org/software/.
- Download the Raspberry Pi Imager tool suitable for your operating system (Windows, macOS, or Linux).
- Install Raspberry Pi Imager on your computer by following the provided instructions.
####Flashing the Raspberry Pi OS Image to USB
After downloading Raspberry Pi OS and installing the Imager, you can flash the OS Image to your USB (SD card can also be used):
 - Launch Raspberry Pi Imager on your machine.
 - In the Raspberry Pi Imager window, click on "Choose OS" button.
 - Click on "Use custom" option, then navigate to the directory of the OS that was previously downloaded.
 - Click on "Choose  Storage" option, in the file explorer window select your USB flash drive as the target storage medium.
 - The widnow should have the same configuration as the image below after choosing the flash drive and OS:
     
      ![MobaXterm](/other/figs/RaspberryPiOS/1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 1: RaspberryPi Imager </i></p>

 - Now click on the settings icon in the bottom right corner, you will need to enable the SSH option and choose a username and a password (This is necessary because you will use SSH to establish a connection between your device and Raspberry Pi, it will allow you to control the Raspberry Pi remotely).

  ![MobaXterm](/other/figs/RaspberryPiOS/2.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 2: RaspberryPi SSH connection enabling </i></p>
  ![MobaXterm](/other/figs/RaspberryPiOS/3.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 3: RaspberryPi username and password </i></p>

- Next, you will need to enter the name of your network and its password, this will allow you to access the Raspberry pi directly and remotely.
   
   ![MobaXterm](/other/figs/RaspberryPiOS/4.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 4: RaspberryPi network configuration </i></p>

- Click on the "Write" button to start the flashing process. Raspberry Pi Imager will download the latest Raspberry Pi OS image and write it to the USB flash drive. This process may take several minutes, depending on your machine.
Once the flashing process is completed, Raspberry Pi Imager will display a success message.

### Raspberry Pi Remote Connection configuration
There three ways to control Raspberry Pi:
- Use a mouse, a monitor and a keyboard attached to your Raspberry Pi.
- Use ethernet cable attached to your machine and Raspberry Pi, and a virutal machine software.
- Connect both Raspberry Pi and your machine to the same network, and use a virtual machine software.

we will be using the third option since it is be a lot easier to control your Raspberry Pi remotely.
1. To control and access the Raspberry Pi, you will need a program to establish wireless connection. MobaXterm is the program that will allow you to establish this communication, which is based on SSH protocol.
 - Download the program from this [link]( https://mobaxterm.mobatek.net/download-home-edition.html), choose portable edition.
 - After succefully downloading the software, extract the contents of the downloaded file to a specified folder.
 - The program is ready to be used now, when launching it, a window like that will appear (it should be noted that both the raspberry pi and your device have to be connected to the same network):

   ![MobaXterm](/other/figs/MobaXterm/1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 5: MobaXterm Interface </i></p>

 - Choose Session tab in the top left corner:

   ![Session Option](/other/figs/MobaXterm/2.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 6: Choosing new session </i></p>

 - A new window will appear that contains the settings of the new connection:

   ![Session window](/other/figs/MobaXterm/3.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 7: new session tab </i></p>

 - Click on the SSH icon, we will be using SSH in our work. SSH (Secure Shell) is a network protocol that provides a secure way to access and manage remote systems. It allows to establish a secure encrypted connection between local computer and a remote device, such as a Raspberry Pi in our case.Using SSH to access a Raspberry Pi offers several advantages, as it allows remote access to the Raspberry Pi's command-line interface, providing a convenient and efficient way to manage and configure the device. Through an SSH connection, commands can be executed, files can be transfered, perform system administration tasks without the need for physical access to the Raspberry Pi. Moreover, SSH is particularly useful for headless Raspberry Pi setups, where the device is not connected to a monitor or keyboard. In such cases, SSH enables remote administration, making it possible to configure and control the Raspberry Pi over the network from a different computer.

   ![SSH](/other/figs/MobaXterm/4.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 8: choosing SSH communication  </i></p>

- To establish a connection with the Raspberry Pi, remte host name will be needed, which will be raspberrypi.local (The IP of the Raspberry Pi can also be used, but the domain name raspberrypi.local will work fine). With current Configuration, all commands will be executed through the terminal, to make these operations easier, LXDE desktop will be used as GUI as in the image below:

   ![Settings Configuration](/other/figs/MobaXterm/6.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 9: Remote Host and LXDE Desktop</i></p>

- Now the settings are ready, click ok and it will need the username and password that was set during the setup of the raspberrypi py operating system:

   ![Settings Configuration](/other/figs/MobaXterm/7.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 10:Login window </i></p>

- After applying the previous steps, connection is succefully established and Raspberry Pi is ready to be used remotely, a new LXDE desktop window will be opened, all subsequent work will be done here:
  
   ![Settings Configuration](/other/figs/MobaXterm/8.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 11: RaspberryPi LXDE desktop </i></p>

- Update Raspberry Pi, enter the following commands in order, this step make take some time:

   1- ```sudo apt update``` 

   2- ```sudo apt upgrade```

### Raspberry Pi Camera Activation

- One last step is to turn on Raspberry Pi camera. Write the command below in the terminal:

   ```sudo raspi-config```

- The window of Raspberry Pi settings will be opened, use arrow keys to move the cursor between the options. Choose the option with label "Interfacing Options" by pressing "Enter" key:

  ![MobaXterm](/other/figs/RaspberryPiOS/cam1.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 12: Interfacing Options </i></p>

- Now choose camera and press "Enter", the camera is activated now.
  
    ![MobaXterm](/other/figs/RaspberryPiOS/cam2.png)

    ![MobaXterm](/other/figs/RaspberryPiOS/cam3.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 13+14:Choosing camera option </i></p>

- After finishing the previous steps, press "ESC" to return to the original terminal, and enter the command below:
 
  ```sudo reboot```

### Installing Arduino IDE on Raspberry Pi

- Arduino IDE installation steps are straightforward. Open the terminal by clicking on the terminal icon or by using the shortcut Ctrl+Alt+T, then enter the following command:

   ```sudo apt install arduino```

- Before using the IDE, setting the permissions for the Arduino port in Raspberry Pi is needed:
Connect the Arduino to the Raspberry Pi using a USB cable. In the terminal on the Raspberry Pi, type the following command to list the available ports on your Raspberry Pi:
  
   ```ls /dev/tty*```

- This command will display a list of devices, and the Arduino port should be listed as something like `/dev/ttyACM0` or `/dev/ttyUSB0`. Note down the port name as you will need it for the next step. By default, the serial ports in Raspberry Pi are not accessible to regular users. To grant access to the Arduino port, you can use the chmod command to change the permissions of the port. Run the following command, replacing /dev/ttyACM0 with the actual port name you obtained:

   ```sudo chmod a+rw /dev/ttyACM0```

- After running the chmod command, you should now have the necessary permissions to access the Arduino port, and you are ready to use the arduino with the raspberry pi. To use the Arduino IDE, you can easily access it in the programming tab as in the image below:
   
   ![Settings Configuration](/other/figs/MobaXterm/34.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 15:Open programming tab and click Arduino icon </i></p>

- A window will open like the one below:

   ![Settings Configuration](/other/figs/MobaXterm/11.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 16:Ardunio IDE interface</i></p>

### Downloading OpenCV PySerial library

- PySerial library is essntial component to establish a succeful connection between Arduino and Raspberry Pi, it can be easily downloaded using the following command below:

   ```sudo apt-get install python-serial```

### Downloading OpenCV library on Raspberry Pi

We will be using OpenCV library, which stands for (Open Source Computer Vision Library). OpenCV is a popular open-source computer vision and machine learning library. It provides a vast collection of functions and algorithms for image and video processing, object detection and tracking, feature extraction, and more.

- OpenCv library isn't installed by default on the Raspberry Pi, you will have to install it. Open the terminal, and type the following command:

   ``` sudo apt-get install python3-opencv```
- Wait till the installation is done, when it is done, open any code editor, which can be found in the programming tab:
  
     ![Settings Configuration](/other/figs/MobaXterm/34.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 17:Open programming tab and click Geany icon</i></p>

- Choose Geany code editor for example. Geany is a simple and easy to use code editor (Try searching the internet for more details in case you want to know more), a window like this will be opened:
  
     ![Settings Configuration](/other/figs/MobaXterm/9.png)<p align="center" Style="font-size:12px; color:grey; font-type:italic;"><i> Figure 18:Geany editor interface </i></p>

- Import the library using the following command, and you are ready to use the library:

   ```import cv2```

---
