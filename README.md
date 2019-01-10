UDS Server - STM32
====================

This branch hosts the ported code of the uds-server to STM32F303K8.
Code mostly WIP at the moment, but it compiles.

Unified Diagnostic Services (UDS) Server - is a ECU simulator that provides UDS support.
This application was originally written to go alongside of [ICSim] (https://github.com/zombieCraig/ICSim)
for training.

Running both ICSim and uds-server can give students a more realistic use of tools.  You can use
ICSim to understand the basics of reversing CAN and uds-server to dig into the UDS protocol
and Engine Control Unit (ECU) inspections such as memory reads and device I/O controls via the ECU instead of spoofed
Controller Area Network (CAN bus) packets.

In addition, when developing uds-server, it showed several more uses.  When a dealership tool
is known to uds-server, it makes it very easy to see what the tool is attempting to do by spoofing
a real target vehicle.  This allows you to quickly reverse commands from dealership tools and
see only the packets that matter.  Another nice feature, is the ability to fuzz the
dealership/scantools to see if they are doing proper input validation checks.  This enables
uds-server to work as a security tool by playing the role of a modified "malicious" vehicle and
seeing how the shop's tools handle the malformed requests.


Then you can practice commands to get VIN or use things like [CaringCaribou] (https://github.com/CaringCaribou/caringcaribou) to brute force or identify diagnostic services.

If you ware working with a dealership tool or a scan tool then you will use the real can0 interface
instead.  You will need a small CAN network to bridge the dealership/scantool with your CAN
sniffer attached to uds-server.  You can breadboard this or build a small portable device we lovingly
call the ODB GW.

Credits
=======
Craig Smith - craig@theialabs.com
OpenGarages - opengarages.org (@OpenGarages)

