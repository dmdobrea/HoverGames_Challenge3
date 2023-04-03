# NXP HoverGames3: Land, Sky, Food Supply

These are the codes developed in the frame of NXP HoverGames Challenge 3: Land, Sky, Food Supply

## Overview

In this project, I have developed an intelligent system to assess the healthiness of vines through the characteristics resulting from the analysis of the vine leaves (texture, shape, or color). 

## Components

### Hardware components:

- 1 x **NXP KIT-HGDRONEK66** (carbon frame kit, BLDC motors, ESCs, PDB, propellers, etc.)
- 1 x **NXP RDDRONE-FMUK66** - flight management unit
- 1 x **NXP 8MPNavQ-4GB-XE** - an embedded computer (i.MX 8M Plus Quad with a Neural Processing Unit, 4GB LPDDR4, WiFi/BT)
- 1 x **Google Coral camera**
- 1 x **NXP HGD-TELEM433** - 433Mhz Telemetry Radio 
- 1 x **Intel RealSense D435i**
- 1 x 4S 5000 mAh battery (3S can work too)

### Software components

#### Official support software

- **PX4** - an open source flight control software for drones and other unmanned vehicles
- **PX4 QGroundControl** - Ground Control Station for the MAVLink protocol

#### Main software components developed for NXP HoverGames Challenge 3

The components that implement the software part of the **agriHoverGames** drone were developed as a single **ROS 2** package consisting of 4 files written in Python:
1. videoPub.py:
2. videoWiFibroadcast.py
3. flightControl.py and
4. healthPlant.py 

ebererb
