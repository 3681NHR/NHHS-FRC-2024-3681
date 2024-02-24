 # Team 3681 Robot Functionality Checklist

note: for full check(after changes), follow every instruction exactaly in the order they are told, if something cannot be confirmed, or is wrong, 
**stop the robot and fix it!**

 ## Core Inspection
 
 * Before powering up the robot, check the following: 
    * confirm all wires in place
    * confirm all loose wires sepperated to avoid short
    * confirm no metal shavings in electronics
    * confirm fuses in place
* Add power 
    * put battery in holder and strap in tightly
    * main breaker ON
* Inspect basic connections and communication
    * confirm router and roboRIO on with no errors
        * Router will take a couple minutes to initialize
            * Look for (TODO) lights to be on 
        * RoboRIO Status light should be on and solid
    * connect computer to RoboRIO
        * Computer joins the (TODO) `3681` network (for the competition robot)
    * On the computer, open Driver Station and Smartdashboard
        * confirm driverstation and smartdashboard connected to robot
            * (TODO) Look at (these) status indicators to confirm
            * On the Smart Dashboard, there is a "Connected" widget that is Green when connected; Red otherwise
    * deploy robot code
        * From VS Code, click the WPI icon and find the "Deploy Robot Code" entry

## Driver Station 

In the Driver Station, confirm the following items: 

 * on driverstation
    * Communications: all green (not partial green)
    * Robot Code: green
        * Indicates we have deployed code to the RoboRIO that can be run
    * Joysticks: green
        * Indicates that joysticks are connected to the laptop
    * Robot battery voltage: >11V
        * Under this voltage, the robot my not perform as expected
    * Mode: teleoperated
        * This is the menu on the left-hand side, where we can select "Autonomous", etc. 
    * Enabled: disable
        * On startup, the robot should be "Disabled" until ready to drive
    * Team number: 3681
    * Joysticks: xbox controller on port 0
        * This is in the USB sub-menu (select on the left-hand side tab menu)
    * Dashboard type: smartdashboard
        * This is in the Setup sub-menu (select on the left-hand side tab menu)

## Run the Robot

 * confirm no one near robot
 * enable robot
    * big green 'Enable' button

### Checking Core Functionality

#### Check drive train
* confirm all 4 wheels moving with user inputs
* confirm speeds consistant/close to input magnatude
* confirm deadzone and limitor working propperly(no drift)
* confirm wheel direction correct
* confirm drive controllers no errors
* confirm drive controllers set to BRAKE idle mode(wheels stop when no input)
* check motion on ground

#### Check launcher motors
* set to DROP speed
    * confirm speed at DROP speed
* set to LAUNCH speed
    * confirm speed at LAUNCH speed
* set to IDLE
    * confirm motors slowing to IDLE

#### Check intake motors
* set to INTAKE speed
    * confirm top and bottom motors at INTAKE speed
* set to REVERSE speed
    * confirm top and bottom motors at REVERSE speed
* set to IDLE
    * confirm motors slowing to IDLE

#### Launcher Homing Sequence
* start launcher homing sequence
    * if expected motions are not happening, STOP HOMING SEQUENCE IMEDIEDLY
    * confirm launcher swing is moving at homing speed tords homing switch
    * confirm launcher swing stops at homing switch and resets encoders to zero
 
#### Intake Homing Sequence
* start intake homing sequence
    * if expected motions are not happening, STOP HOMING SEQUENCE IMEDIEDLY
    * confirm intake swing is moving at homing speed tords homing switch
    * confirm intake swing stops at homing switch and resets encoders to zero

#### Launcher Swing: Manual Control
* confirm launcher swing moving up/down with inputs
* confirm propper speeds moving up/down
* confirm propper deadzone and limitor working propperly

#### Launcher Swing: PID/Positional Hold
* move arm to low position without resting on bumper or lim. switch
    * confirm arm is holding steady in place for ~5 seconds
* move arm to medium position
    * confirm arm is holding steady in place for ~5 seconds
* move arm to high position
    * confirm arm is holding steady in place for ~5 seconds

#### Launcher swing: autoset position hold
* set position to autoset LAUNCH position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for ~5 seconds
* set position to autoset RECV position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for ~5 seconds
* set position to autoset DROP position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for ~5 seconds

#### Intake swing: autoset positional hold
* set position to autoset UP position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for ~5 seconds
* set position to autoset DOWN position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for ~5 seconds

#### Auto Recieve function: without gamepiece
* start auto recv
    * confirm launcher arm moves to RECV position
    * confirm intake arm moves to UP position
    * confirm launcher and intake arms line up
* confirm intake top and bottom motors set to REVERSE speed
* confirm launcher recever motor set to INTAKE speed

#### Auto Recieve function: with gamepiece
* use intake to collect gamepiece
* Start auto recv
    * confirm launcher arm moves to RECV position
    * confirm intake arm moves to UP position and held game piece is not impacting anything and is held firmly
    * confirm launcher and intake arms line up
    * confirm intake top and bottom motors set to REVERSE speed
    * confirm launcher recever motor set to INTAKE speed
    * confirm gamepiece moves to launcher assembaly, not touching launcher wheels or intake
