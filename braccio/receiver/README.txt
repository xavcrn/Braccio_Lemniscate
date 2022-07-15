This receive file is used to read the control comming from the Egg Controler.
It reads the data comming the RF24 SPI module and push it on a the temp/transferToPython pipe.
Then the pipe is read by receiver.py script that send the data to pypot_controler.py across a ROS topic.
(note : you need to create this pipe using mkfifo command before trying to run this program)
Of course it is better to read the RF24 data and to publish it on a ROS topic on the same program but I didn't know how to use the RF24 libraries with ROS catkin_make so I have compiled it apart with g++ and then put the file here.
Maybe I will make it better one day.