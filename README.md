# ETSI DCC Access implementation for ns-3

This is a software module for the network simulator ns-3 that implements the DCC (Decentralized Congestion Control) Access protocol specified by ETSI in [TS 102 687 v1.2.1](https://www.etsi.org/deliver/etsi_ts/102600_102699/102687/01.02.01_60/ts_102687v010201p.pdf). DCC controls the data traffic injected by each vehicle to the radio channel for ITS-G5. It makes use of Prioritization, Queuing and Flow Control. This module has been tested with ns-3.26 and ns-3.20, but can be easily integrated in other versions of ns-3. We plan to integrate it into the iTETRIS simulation platform, which is being updated and evolved in the [H2020 TransAID](https://www.transaid.eu/) research project.

In order to comply with our sponsor guidelines, we would appreciate if any publication using this software references the following publication:

    Miguel Sepulcre, Jorge Mira, Gokulnath Thandavarayan, Javier Gozalvez,
    "Is Packet Dropping a Suitable Congestion Control Mechanism for Vehicular Networks?", 
    Proc. IEEE 91st Vehicular Technology Conference (VTC2020-Spring), Antwerp, Belgium, 25-28 May 2020. 

What DCC parameters can be modified:

    Algorithm: Reactive, Adaptive or NoDCC
    CBR target: only for Adaptive approach
    Queue length
    Packet life time

How to install:

    1.- Simply copy the "dcc" folder into your "src" folder.
    2.- Configure and compile ns-3 following the normal procedure.
    
How to run the example script available in the "scratch" folder:
    
    ./waf --run "dcc-example --Algorithm=Reactive --simulationTime=20 --NumCars=200 --Speed=27"
