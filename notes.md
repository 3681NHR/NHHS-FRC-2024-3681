this is a list of notes about the code/interesting things in said code

 ## cool things
 * input squaring
    * when controlling things like a drivetrain, you want to be able to have the full range of movement,
    but also need high precision at low speeds. 
    * when you square the input from a joystick, the robot's motion will be precise when at low speeds, but you will still have the full range of speed
    * this graph shows input squaring
    * ![input graph](image.png)
        * the black shows a liner relation between user input and output speed
        * the red is after squaring the input and reaplying the sign(multiply by x/|x|)
        * the blue is after cubing the input
    * when you square(or cube) the input from the joystick, your drive will have better percision at low speeds, without loosing your top speed
    
 * feedback-feedforward combined control
    * a feedback loop uses mesured data t oaccount for error
    * a feedforward control system will use a model to estimate how it should move motors
    * the problems
        * feedback loops are by definition, reactive, meaning it only moves to react, making it delayed and incapable of accounting for things like gravity, that will apply a constant force

        * a feedforward system cannot react to any error as it defines input based on a model, and since the model will never be 100% accurate, the feedforward will be inaccurate is velocity, and small velocity errors will add up to large error in position, making feedforward-only a bad idea on arms

     * pros
        * feedback systems wont drift over time and are very accurate
        * feedforward will usa a model, allowing it to predict and account for things like gravity BEFORE it affects the system you are controlling

    * combine them!
        * by combining feedback and feedforward, you can make a system that has a constant "hold" output and also reactive control
        * when a setpiont is given, the feedforward can use its model to account for gravity, static friction, and shifting voltage-motion relationships
        * the feedback system can react to the differences brtween the model and reality
        * this combination will allow a system to maintain a position and react, while still predicting future error from things like gravity, and reacting before its a problem

 * trapazoid controll
    * when using a controller(like PID) to move something, the simple attack woult be to just set the setpoint directly where you want the system to go to
    * trapazoid control will interpalate between multiple points that will bring the system to its desired state is a faster, more controlled manner

 * 