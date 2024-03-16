# Running the Robot via the Simulator

It seems like it would be super useful to run the robot via the simulator before we can run it for real, but the simulator is challenging to figure out how to get running properly. 

Collecting notes on how to make it work/run: 

* When starting the simulator, when ready, find the "Robot State" window, and set the State to something like "Teleoperated" to test the driving, or "Autonomous" to test the autonomic code
* Simulating the XBox controller: 
    * If you don't have an XBox controller or Joystick plugged into the simulating machine, specify a keyboard to be used for input. 
        * Find the "System Joysticks" window. 
        * Find an entry that is not null (might say something like "Keyboard X")
        * Drag your desired Keyboard entry to the "Joysticks" window, under the ID that your XBox controller requires
            * If the XBox Controller is expected to be on USB 0, move to the "Joystick[0]" entry
        * Use the S, D, R and E keys to "move the joystick"

* Right now: 
    * It's showing an update on the field because the encoders and the gyro and just calculating numbers and going up. 
    * What is supposed to happen is that when the input controller moves, it moves the drive component, which is supposed to update the motors, which then should update the encoders. In reality, the gyro gets updated because it actually moves as a result of the movement. 
        * To simulate the movement of the gyro, i guess we want to calculate which direction the robot should be facing. 
    * We can make it more realistic by 



* What do we want to simulate? 
    * When the joystick is [one way] the robot moves forward
    * When the joystick is the other way, the robot moves backgard
    * When it is another way, it rotates
    * I think it means that in the Drive() command, we update positions. 
        * What we expect is that if the drive command is going, it updates the motors, which updates the encoders, which makes it look like the robot is moving
    * In the simulation, the encoder values are increasing because we keep calling get(), which (hackily) updates the position
    * 