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

