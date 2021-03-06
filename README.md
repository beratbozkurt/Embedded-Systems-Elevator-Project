# Embedded Systems - Elevator Project

## OBJECTIVE OF THE PROJECT
In this project, it was planned to build an elevator system on the stm32 card using the
information explained in the embedded systems course. Due to the use of various sensors and
actuators, the system, which was prepared in the simulation environment, became more
concrete. As a result, the system was made to work in accordance with the logic of real life
by giving various responses to various inputs.

## THE DESIGN
The system contains 3 different LEDs for the location information of the elevator, and 3
different buttons have been added for requests by the users for each floor as well. In addition,
2 different LEDs were added for the up and down movement of the elevator and 2 different
LEDs for the open and closed movement information of the door. While the added engine
provides the movement of the elevator, the elevator movements can be followed with the
LCD. An ultrasonic sound wave sensor was added between the door, and the door's
movement was regulated accordingly. It is also planned to send a warning sound to inform
users when the door is closing. Gas and flame sensor has also been added, in case of any
danger, the alarm is given with a buzzer. The algorithm was designed according to the
frequency of use of the elevator of the floors. Inputs by the admins over the virtual terminal
were set, and floor information and sensor information were delivered to the virtual terminal.
Finally, the speed of the elevator was adjusted with the analog input.

See full version of the report [here](https://github.com/beratbozkurt/Embedded-Systems-Elevator-Project/blob/main/EE304_ProjectFinalReport_Berat_Bozkurt_Emrullah_Dagkusu.pdf).

![image](https://user-images.githubusercontent.com/56932641/175398635-c330cb34-48b7-41d3-8fcb-6e99666dcf30.png)
