# Arduino firmware
Both Programming USB and Native USB of Arduino are used. 
- if useNativeUSB is true, the data packages will be sent to the Native USB port (next to reset button) of Arduino, otherwise to Programming port (next to big black power connector).
- if useNativeUSB = true and debug = true some diagnostics messages will be sent to Programming USB port.
