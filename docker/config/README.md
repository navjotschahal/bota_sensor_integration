# Driver configs

One JSON per communication interface, in the format the official Bota Gen A
driver expects (`"driver_config"` object). Select one with `BOTA_CONFIG`.

| file | interface | mandatory param | notes |
|---|---|---|---|
| bota_socket.json | Ethernet, Bota Socket | `sensor_ip_address` | Gen A Industrial/ENET sensor, streaming |
| ethercat.json | CANopen over EtherCAT, Gen A | `network_interface` | needs NET_RAW, polled |
| ethercat_gen0.json | CANopen over EtherCAT, Gen 0 | `network_interface`, `sinc_length` | Rokubi/SensONE Gen 0 |
| bota_binary.json | USB / RS422 | `com_port` | **this lab's sensor** (BFT-SENS-IND2-B over USB-C CDC-ACM); device passed through by compose |

`app_mode` / `app_submode` set the sensor's operation mode and update rate.
The authoritative values come from Bota's online config tool
(https://shop.botasys.com/config-tool, "Operation" tab, "Download Config"),
which writes exactly this JSON. Replace the files here with the downloaded one.
`product_name` and `serial_number` are informational.
