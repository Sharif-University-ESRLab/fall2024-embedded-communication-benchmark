
<img src="https://www.sharif.ir/documents/20124/0/logo-fa-IR.png/4d9b72bc-494b-ed5a-d3bb-e7dfd319aec8?t=1609608338755" width="150px" />


# Creating an embdedded system's benchmark (communication)

This project was done for a hardware lab module in sharif university of technology. The purpose of the project is creating a benchmark for testing communication protocols like UART, I2C, and SPI in embedded systems. The implementation was done with the ESP32 board in mind and the chosen coding platfrom was ESP-IDF.


## Tools
In this section, you should mention the hardware or simulators utilized in your project.
- ESP32
- ESP-IDF


## Implementation Details

In this section, you will explain how you completed your project. It is recommended to use pictures to demonstrate your system model and implementation.


Feel free to use sub-topics for your projects. If your project consists of multiple parts (e.g. server, client, and embedded device), create a separate topic for each one.

## How to Run

- Follow the installation link to install the ESP-IDF framework. From this point forward, we will assume you have installed the framework under ~/esp/esp-idf/

- Activate environment by running
```bash
~/esp/esp-idf/export.sh
 ```
if you run into any problems at this step, please check out the installation toturial provided by the framework.

- Clone the repository and navigate to the project folder

- If this is your first time running this code, run
```bash
python idf.py set-target esp32
```

- Run 
```bash
python idf.py menuconfig
```
and configure the project. Be sure to save when you're done.

- Make sure the connections on the board are all correct, and conncet the board to your computer

- Look in the /dev folder to find your esp32 device (typically in a directory like /dev/ttyUSB0)

- Run
```bash
python idf.py -p {block device address} flash monitor
```
(You may need to run chmod to allow idf.py to access the block device)


## Results
As expected, the SPI protocol vastly outperforms its counterparts. The read time is not comparable due to the nature of the SPI protocol, but we can compare and plot the write times. <br />
- UART results <br />
<img src="./Miscellaneous/UART.jpg" />
- SPI results <br />
<img src="./Miscellaneous/SPI.jpg" />
- I2C results <br />
<img src="./Miscellaneous/I2C.jpg">

This plot aptly illustrates the results and how they compare: <br />
<img src="./Miscellaneous/results.png" />


## Related Links
Some links related to your project come here.
 - [esp-idf installation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html)
 - [esp-idf i2c](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/i2c.html#introduction)
 - [esp-idf spi master](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/spi_master.html)
 - [esp-idf spi slave](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/spi_slave.html)
 - [esp-idf uart](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/uart.html)


## Authors
Authors and their github link come here.
- [@NegarAskari](https://github.com/NegarAskari)
- [@HiradDavari](https://github.com/theAester)

