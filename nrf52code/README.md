main.c is the main code

sdk_config.h is the config file

.uvprojx is the project with the correct packs in order to run the example

The hex file can be loaded onto the nrf52832 MCU and run

The Keil uVision project is adapted from the SAADC peripheral example from the nRF5 SDK v16.0.0. The main.c and the sdkconfig.h files are the only two files that were modified

The project can be found in saadc_pca10040.uvprojx. This was developed using the PCA10040 development board, which has the nRF52832 SoC, and can be found \saadc\pca10040\blank\arm5_no_packs.

It is recommended that the full nRF5 SDK v16.0.0 is available from your working directory. The SDK can be downloaded from here: https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.0.0%2Findex.html, which also contains a guide to getting started. 

This was developed by combining the saadc and PWM Library examples from the Nordic SDK
