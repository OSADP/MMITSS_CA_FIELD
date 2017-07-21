# About

### Application Name: MMITSS-CA

### Version Number: v2.0

### Summary
Multi-Modal Intelligent Traffic Signal System (MMITSS) is the next generation of traffic signal systems
that seeks to provide a comprehensive traffic information framework to service all modes of transportation,
including general vehicles, transit, emergency vehicles, freight fleets, and pedestrians and bicyclists 
in a connected vehicle (CV) environment.

The MMITSS applications bundles includes the Arizona version of MMITSS (MMITSS-AZ) and the California version of MMITSS (MMITSS-CA).
The MMITSS-AZ aims to provide adaptive traffic and signal priority control using connected vehicle data. The MMITSS-CA aims to improve 
the existing actuated-coordinated traffic signal control system, with performance enhancements and signal coordination and priority
control techniques enabled by the connected vehicle data. Traffic control agencies are provided with options for selective use of 
MMITSS-AZ or MMITSS-CA based on their needs. This package includes software set for the MMITSS-CA. The MMITSS-AZ software set can be
download from [OSADP](https://www.itsforge.net/index.php/community/explore-applications/for-search-results#/30/63).

The MMITSS-CA software set includes software to be hosted at three (3) physical devices:
-	An Ubuntu Linux-based MMITSS Roadside Processor (MRP) that generates SAE J2735 MAP, SPaT (Signal Phase and Timing)
and SSM (Signal Status Message) messages, processes SAE J2735 BSM (Basic Safety Message) and SRM (Signal Request Message) messages,
calculates performance measures, determines MMITSS traffic and signal priority control strategies, and communicates control commands 
to the traffic signal controller;
-	A messages transceiver software module running on an RSU (RoadSide Unit) device that routes outbound SAE J2735 MAP, SPaT and SSM
messages to be broadcasted over-the-air, and routes inbound over-the-air SAE J2735 BSM and SRM messages to the MRP computer; and
-	Software modules running on an OBU (On-Board Unit) device that generate and broadcast SAE J2735 BSM and SRM messages, 
and receives and process SAE J2735 MAP, SPaT and SSM messages.

### Description
The MMITSS-CA software includes three (3) arterial traffic signal applications:
-	**Intelligent Signal Control (I-SIG)** 
  
  I-SIG provides CV-based signal actuation and dilemma zone protection. The CV-based coordination control will be included 
	in the future release of MMITSS-CA. I-SIG takes vehicle trajectory data from BSMs, places a service call to the controller 
	on the phase that controls the vehicle's movement, and when needed, extends the green phase for dilemma zone protection. 
	The phase call and extension control commands are communicated with the traffic controller through AB3418 over RS-232 communications. 
	Interfacing with NTCIP controllers for MMITSS-CA traffic and priority control will be included in the future release of MMITSS-CA.
	
-	**Signal Priority (SP)**

  Signal priority provides priority control to different modes of vehicles including transit vehicles and trucks. 
	Priority eligible vehicles (e.g., OBUs) receive SAE J2837 MAP, SPaT and SSM from RSU when they enter the DSRC communication 
	range and generate and broadcast SRMs. The MRP receives and processes the SRMs, considers all the active priority requests 
	from different vehicles to decide the appropriate priority treatment, and communicates the control command to the traffic 
	signal controller. The adaptive signal priority feature that solves for optimal priority green splits based on active priority 
	requests and prevailing traffic condition in terms of minimization of weighted traffic and priority vehicle delay will be included 
	in the future release of MMITSS-CA. 
	
- **Mobile Accessible Pedestrian Signal System (PED-SIG)**

  PED-SIG works together with [Savari SmartCross Application](http://savari.net/solutions/smart-phone/), which includes 
	a pedestrian cloud sever that receives SAE J2735 MAP and SPaT from connected RSUs, and pedestrian phase request from 
	SmartCross phone app. The MRP receives and processes the pedestrian SRM, and places a pedestrian service call to the 
	traffic signal controller.

# Release Notes

### License
- MMITSS-CA software is licensed under the Educational Community License vervison 2.0 ([ECL](https://opensource.org/licenses/ECL-2.0)). 
- ECL-2.0 consists of the [Apache 2.0 license]( http://www.apache.org/licenses/LICENSE-2.0), modified to change the scope of the patent 
grant in section 3 to be specific to the needs of the education communities using this license.
- MMITSS-CA makes use of the open source ASN.1 compiler ‘[asn1c](https://github.com/vlm/asn1c)’, which is licensed under 
BSD 2-Clause ("[Simplified BSD License](https://opensource.org/licenses/BSD-2-Clause)").  

### SAE J2735 Meessage Set Definition
- [Version 2016-03](https://www.sae.org/standardsdev/dsrc/)

### Subdirectory and Content
 Subdirectory    | Host Location           | Contents
 ----------------|-------------------------|----------
 **mrp**         | Ubuntu Linux computer   | MRP software components, including MRP_TCI, MRP_DataMgr, MRP_Aware, and source code for creating shared dynamic libraries: libasn, libdsrc and liblocAware
 **savari/rsu**  | Savari StreetWAVE (RSU) | RSU software component RSU_msgTranciver (Message Tranciver) 
 **savari/obu**  | Savari MobiWAVE (OBU)   | OBU software components, includig OBU_msgTranciver and OBU_Aware  
 
### Installation Instructions
The installation instructions for MRP components on the Ubuntu Linux computer, RSU components on Savari StreetWAVE devices, 
and OBUS components on Savari MobiWAVE devices are described in README under the directory ‘[mrp](mrp/README.md)’, 
‘[savari/rsu](savari/rsu/README.md)’, and ‘[savari/obu](savari/obu/README.md)’, respectively.

### Operating Requirements
- MRP Ubuntu Linux Computer
    - Processing power:  Intel Core i5 or equivalent
    - Minimum memory:    2 GB
    - Hard drive space:  100 GB or above
    -	Connectivity:      Ethernet, 2 RS232 ports
    - Operating system:  Ubuntu 16.02, Linux Kernel 4.08 or above
- Savari StreetWave and MobiWAVE devices
    - Savari On-Board Operating System (SOBOS) version 3.1.2 or above

# Documentation
- MMITSS project background and detailed documentations is available through the 
[Connected Vehicle Pooled Fund Study website](http://www.cts.virginia.edu/cvpfs_research/)
- [MMITSS Concept of Operations](http://www.cts.virginia.edu/wp-content/uploads/2014/05/Task2.3._CONOPS_6_Final_Revised.pdf)
- [MMITSS-CA Detailed System and Software Design](http://www.cts.virginia.edu/wp-content/uploads/2014/04/32-MMITSS-Phase-2-Detailed-Design-CA-final.pdf)