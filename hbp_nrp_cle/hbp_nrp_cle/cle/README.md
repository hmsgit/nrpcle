# Description 

The package contains the implementation of the Closed Loop Engine.

The CLE interface grants complete control over both the physic and the
neural simulations. The two simulations can be started, advanced, paused and
resetted in a synchronous manner.

The following implementation of the CLE interface is given:

    ClosedLoopEngine is an implementation that overcomes the NEST bug
    by running the neural simulation and the transfer functions in the same
    thread. The physics simulation is triggered in a separate thread while the neural simulation
    and the transfer functions run in the same thread sequentially.

The ROSCLEServer module provides utility classes to run the Closed Loop
Engine in a separate process, while communicating with it through ROS services.
