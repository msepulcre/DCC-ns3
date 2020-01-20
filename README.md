# ETSI DCC Access implementation for ns-3

This is a software module for the network simulator ns-3 that implements the DCC (Decentralized Congestion Control) Access protocol specified by ETSI in TS 102 687 v1.2.1. DCC controls the data traffic injected by each vehicle to the radio channel for ITS-G5. It makes use of Prioritization, Queuing and Flow Control. This module has been tested with ns-3.26 and ns-3.20, but can be easily integrated in other versions of ns-3. We plan to integrate it into the iTETRIS simulation platform, which is being updated and evolved in the H2020 TransAID research project.

In order to comply with our sponsor guidelines, we would appreciate if any publication using this data references the following publication:

    Miguel Sepulcre, Jorge Mira, Gokulnath Thandavarayan, Javier Gozalvez,
    "Analysis of the Application-Level Impact of ETSI DCC Access  in Vehicular Networks", 
    Submitted to IEEE Vehicular Technology Conference (VTCSpring 2020) (in reviewing process, to be updated)

What DCC parameters can be modified:

    Algorithm: Reactive, Adaptive or NoDCC
    CBR target: only for Adaptive approach
    Queue length
    Packet life time

How to install:

    Symply copy the "dcc" folder into your "src" folder.
    
How to run the example script available in the "scratch" folder:
    
    ./waf --run "dcc-example --Algorithm=Reactive --simulationTime=20 --NumCars=200 --Speed=27"
