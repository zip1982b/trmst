# wifi termostat progect

 
* This code will show you how to use I2C module by running task on i2c bus:
 
    * read\write registers external i2c gateway, here we use a ds2482-100.
    * Use one I2C port(master mode) to read or write the other slave.
 
* Pin assignment:
 
    * slave :
        * ds2482-100 - i2c/1-wire gateway
        * ssd1306 oled display
    * master:
        * GPIO18 sda as the data signal of i2c master port
        * GPIO19 scl signal of i2c master port
 
* Connection:
 
    * connect GPIO18 with sda slaves
    * connect GPIO19 with scl slaves
    * external pull-up resistors, driver will disable internal pull-up resistors.
 
* ENCODER:
 
    * gpio33 clk enc
    * gpio25 dt enc
    * gpio26 sw enc 
