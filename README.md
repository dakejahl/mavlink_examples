### Setup
Just change the serial port string to whatever your FTDI device instantiates as. It is currently hard coded as such:
```
std::string _serial_node = "/dev/ttyUSB0";
```

### Build
```
./build.sh
```
### Run
```
./example
```
You should see this every single time. The order in which you boot things does not matter. When restarting this example program, you must give QGC ~5 seconds so that it detects this Fake Camera as "timed out". Once it times out, it is removed from the Known Cameras list in QGC and upon reconnecting it will initiate the Camera Protocol again.
````
setup_port() success!
Starting receive thread
Receive thread started!
Connected!
sysid: 254
MAV_CMD_REQUEST_CAMERA_INFORMATION
send_camera_information_message()
Got MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST
MAV_CMD_REQUEST_CAMERA_SETTINGS
send_camera_settings_message()
MAV_CMD_REQUEST_STORAGE_INFORMATION
send_storage_information_message()
MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
send_camera_capture_status_message()
Got MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
Got MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
Got MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ
```
