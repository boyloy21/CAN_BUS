# CAN_BUS(CONTROL AREA NETWORK)
CAN is communicate protocol that allows devices to exchange data in a reliable and efficient way.CAN bus uses a message-based protocol where devices, called nodes, send and receive messages called data frames. The CAN bus prioritizes data based on ID, so that the most important information is transmitted first. 
## For Process my code is :
1. I use STM32F407VGT6 board in Main_board_Buffalo file to transmit value of sensor to PC using ROS2 in file can_control_Motor.py
2. I use STM32F103CBT6 2 board in file (Motor1_2_Buffalo and Motor3_4_Buffalo) to transmit data read speed Motor and Recieve data control speed Motor from can_control_Motor.py

## For Detail to use:
[Please download this book to follow in step for easy to use](https://github.com/boyloy21/Researched-documents/blob/main/STM32_Tutorial.pdf)
