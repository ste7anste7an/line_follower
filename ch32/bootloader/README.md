\# Bootloader



\## how to use



* load bootloader.bin using wchiptool via USB (hold boot0 while pressing reset to put in flash mode)
* execute copy\_link\_0x1000.bat
* in Arduino: 

&nbsp;	→ sketch → export compiled binary

&nbsp;	→ show sketch folder

* in C:\\Users\\...\\...\\i2cMinimalSlave\\build\\WCH.ch32v.CH32V20x\_EVT find .bin



```

python uf2conv.py <Arduino sketch directory>\\build\\WCH.ch32v.CH32V20x\_EVT\\i2cMinimalSlave.ino.bin -o i2c\_sensor.uf2 -c -f  0x699b62ec -b 0x08001000

```



* press reset twice quickly
* copy .uf2 file to CH32V UF2 drive
