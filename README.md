# A Low-cost Sensor Module for In-building Monitoring and Threat Detection

**Youchao Wang, Clare Hall, yw479**

## Project summary
The project aims to develop a low-cost hardware system for in-building environmental monitoring and potential threat detecting [1]. It provides an entry-level prototype for highly integrated low-cost monitoring system suitable for large-scale deployment.

Three main contributions, both research- and commerical-wise are listed as follows. (1) Acquire data and knowledge for in-building environmental factors and energy matters [2]. (2) Based on the abundant data collected, enable the potential for better building structural design and improve the daily operational strategies [2]. (3) Lower system cost can potentially lead to higher deployment rates for not only research but also commercial use [1][2].

As stated in [1], this project development would eventually benefit those who are working in areas like building security, management and estates. It can also provide a low-cost monitoring system solution for researchers focusing on in-building environmental performance analyses [2].

The project to date has demonstrated that a highly-integrated low-cost environmental monitoring system suitable for in-building deployment can be designed and produced. This unfolds the great potentials of having multiple low-cost sensors and embedded system solutions deployed on-site. Low-cost sensors with less accurate and precise readings, as compared to the high-end products, implemented within the system could be deployed in vicinity of each other. This provides the chance by applying advanced signal processing techniques to cross-validate the sensor readings among different deployed platforms, in order to counteract the effect of receiving less satisfying outputs.

## Repository layout

The system firmware is built based upon `Warp-firmware`, made available to public by `Physical Computation Laboratory, Cambridge`.

In order for the project to work, it is essential to do the following.

        cd ./Warp-firmware-master/build/ksdk1.1.0

and then type in

        ./build.sh

to build the project.

To run the system, use the following commands

        /Applications/SEGGER/JLink/JLinkExe -device MKL03Z32XXX4 -if SWD -speed 4000 -CommanderScript ../../tools/scripts/jlink.commands

Please follow the [link](https://github.com/physical-computation/Warp-firmware) for full information on how to configure `Warp-firmware`.

The raw data for measurements are stored in the `/Data` folder, these are captured using the developed system as well as Salae logic analyser, refer to the final report for more [4].

The `/Reference and Manual` folder contains datasheets as well as references [1] and [2].

## Reference
[1] Y. Wang, “A Low-cost Sensor Module for In-Building Monitoring and Threat Detection Interim Report for
4B25 Embedded Systems,” tech. rep., Cambridge, 2018.

[2] Y. Wang, “A Low-cost Sensor Module for In-Building Monitoring and Threat Detection Project Proposal
for 4B25 Embedded Systems,” tech. rep., Cambridge, 2018.

[3] Monnit, “Monnit Site,” 2018. [Online]. Available: https://www.monnit.com/. [Accessed: 2019-01-13].

[4] Y. Wang, “A Low-cost Sensor Module for In-Building Monitoring and Threat Detection Final Report
for 4B25 Embedded Systems,” tech. rep., Cambridge, 2018.
