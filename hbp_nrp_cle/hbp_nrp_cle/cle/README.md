# Description 

The package contains the implementation of the Closed Loop Engine.

The CLE interface grants complete control over both the physic and the
neural simulations. The two simulation can be started, advanced, paused and
resetted in a synchronous manner.

Two implementation of the CLE interface are given:

    ClosedLoopEngine is a fully parallel implementation in which the physics
    simulation, the neural simulation and the transfer functions run in
    separate threads. Currenty, a bug in NEST makes it impossible to use
    this implementation as running the neural simulation in a separate thread
    causes a segmentation fault.
    
    SerialClosedLoopEngine is an implementation that overcomes the NEST bug
    by running the neural simulation and the transfer functions in the same
    thread, obviously reducing the performances.

The ROSCLEWrapper module provides utility classes to run the Closed Loop
Engine in a separate process, while comunicating with it through ROS services.
