 * confirm all wires in place
 * confirm all loose wires sepperated to avoid short
 * confirm no metal shavings in electronics
 * confirm fuses in place

 * put battery in holder and strap in tightly
 * main breaker ON

 * confirm router and roboRIO on with no errors
 * connect to computer
 * open driver station and smartdashboard
 * confirm driverstation and smartdashboard connected to robot
 * deploy robot code
----
 * on driverstation
    * connected: all green
    * robot code: green
    * joysticks: green
    * robot battery voltage: >11V
    * mode: teleoperated
    * enabled: disable
    * team number: 3681
    * joysticks: xbox controller on port 0
    * dashboard type: smartdashboard
----
 * confirm no one near robot
 * enable robot
----
 * check drive train
    * confirm all 4 wheels moving with user inputs
    * confirm speeds consistant/close to input magnatude
    * confirm deadzone and limitor working propperly(no drift)
    * confirm wheel direction correct
    * confirm drive controllers no errors
    * confirm drive controllers set to BRAKE idle mode(wheels stop when no input)
    * check motion on ground
----
 * check launcher motors
    * set to drop speed
    * confirm speed at drop speed

    * set to launch speed
    * confirm speed at launch speed

    * set to idle
    * confirm motors winding down to idle
----
 * check intake motors
    * set to intake
    * confirm top and bottom motors at intake speed

    * set to reverse
    * confirm top and bottom motors at reverse speed

    * set to idle
    * confirm motors slowing to idle
----
 * start launcher homing sequence
    * if expected motions are not happening, STOP HOMING SEQUENCE IMEDIEDLY
    * confirm launcher swing is moving at homing speed tords homing switch
    * confirm launcher swing stops at homing switch and resets encoders to zero
----
 * start intake homing sequence
    * if expected motions are not happening, STOP HOMING SEQUENCE IMEDIEDLY
    * confirm intake swing is moving at homing speed tords homing switch
    * confirm intake swing stops at homing switch and resets encoders to zero
----
 * check launcher swing manual controlls
    * confirm launcher swing moving up/down with inputs
    * confirm propper speeds moving up/down
    * confirm propper deadzone and limitor working propperly
----
 * check launcher swing pid/positional hold
    * move arm to low position without resting on bumper or lim. switch
    * confirm arm is holding steady in place for 5 seconds

    * move arm to medium position
    * confirm arm is holding steady in place for 5 seconds

    * move arm to high position
    * confirm arm is holding steady in place for 5 seconds
----
* check launcher swing autoset positional hold
    * set position to autoset LAUNCH position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for 5 seconds

    * set position to autoset RECV position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for 5 seconds

    * set position to autoset DROP position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for 5 seconds
----
* check intake swing autoset positional hold
    * set position to autoset UP position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for 5 seconds

    * set position to autoset DOWN position
    * confirm selected position is set
    * confirm arm moving tords set position
    * confirm arm stays as position for 5 seconds
----
 * check auto recv function without gamepiece
    * start auto recv

    * confirm launcher arm moves to RECV position
    * confirm intake arm moves to UP position
    * confirm launcher and intake arms line up

    * confirm intake top and bottom motors set to REVERSE speed
    * confirm launcher recever motor set to INTAKE speed
----
 * check auto recv function with gamepiece
    * use intake to collect gamepiece

    * start auto recv

    * confirm launcher arm moves to RECV position
    * confirm intake arm moves to UP position and held game piece is not impacting anything and is held firmly
    * confirm launcher and intake arms line up

    * confirm intake top and bottom motors set to REVERSE speed
    * confirm launcher recever motor set to INTAKE speed

    * confirm gamepiece moves to launcher assembaly, not touching launcher wheels or intake