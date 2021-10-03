#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include<signal.h>

/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include <mavlink.h>

#include <string>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>

static constexpr uint8_t QGROUNDCONTROL_SYS_ID = 255;
static constexpr uint8_t AUTOPILOT_SYS_ID = 1;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define SUCCESS_CONSOLE_TEXT "\033[32m" // Turn text on console blue

uint64_t microsSinceEpoch();
bool setup_port();
void set_new_datagram(char* datagram, unsigned datagram_len);
void start_recv_thread();
void stop();
bool parse_message();
void send_command_ack_message(uint16_t command);
bool send_message(const mavlink_message_t& message);
void handleCommandLong(const mavlink_message_t& message);

//  Variables
int _fd = -1;
std::string _serial_node = "/dev/ttyUSB0";

std::mutex _send_mutex;
std::thread* _recv_thread = nullptr;
std::queue<mavlink_message_t> _msg_q;

bool _should_exit = false;
bool _connected = false;

uint8_t _channel = 0;
char* _datagram = nullptr;
unsigned _datagram_len = 0;

mavlink_message_t _last_message = {};
mavlink_status_t _status = {};

int32_t _image_count = 0;

void sig_handler(int signum)
{
    if (signum == SIGINT) {
        printf("SIGINT\n");
        stop();
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT,sig_handler); // Register signal handler

	bool success = setup_port();

	if (!success) {
		printf("setup_port() failed!\n");
		return -1 ;
	} else {
		printf("setup_port() success!\n");
	}

    start_recv_thread();

    while (!_should_exit) {
        mavlink_message_t message;

        mavlink_msg_heartbeat_pack(
            AUTOPILOT_SYS_ID,
            MAV_COMP_ID_CAMERA,
            &message,
            MAV_COMP_ID_CAMERA,
            MAV_AUTOPILOT_INVALID,
            0,
            0,
            0);

        // Send heartbeat
        if (_connected) {
            // std::cout << "sending heartbeat..." << std::endl;
            send_message(message);
        }

        sleep(1);
    }
}

uint64_t microsSinceEpoch()
{
	struct timeval tv;
	uint64_t micros = 0;
	gettimeofday(&tv, NULL);
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	return micros;
}

bool setup_port()
{
    // open() hangs on macOS or Linux devices(e.g. pocket beagle) unless you give it O_NONBLOCK
    _fd = open(_serial_node.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd == -1) {
        std::cout << "open failed: " << std::endl;
        return false;
    }
    // We need to clear the O_NONBLOCK again because we can block while reading
    // as we do it in a separate thread.
    if (fcntl(_fd, F_SETFL, 0) == -1) {
        std::cout << "fcntl failed: " << std::endl;
        return false;
    }


    struct termios tc;
    bzero(&tc, sizeof(tc));

    if (tcgetattr(_fd, &tc) != 0) {
        std::cout << "tcgetattr failed: " << std::endl;
        close(_fd);
        return false;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CRTSCTS);
    tc.c_cflag |= CS8;

    tc.c_cc[VMIN] = 0; // We are ok with 0 bytes.
    tc.c_cc[VTIME] = 10; // Timeout after 1 second.

    tc.c_cflag |= CLOCAL; // Without this a write() blocks indefinitely.

    const int baudrate = B115200;


    if (cfsetispeed(&tc, baudrate) != 0) {
        std::cout << "cfsetispeed failed: " << std::endl;
        close(_fd);
        return false;
    }

    if (cfsetospeed(&tc, baudrate) != 0) {
        std::cout << "cfsetospeed failed: " << std::endl;
        close(_fd);
        return false;
    }

    if (tcsetattr(_fd, TCSANOW, &tc) != 0) {
        std::cout << "tcsetattr failed: " << std::endl;
        close(_fd);
        return false;
    }

    return true;
}

void receive()
{
    std::cout << "Receive thread started!" << std::endl;

    // Enough for MTU 1500 bytes.
    char buffer[2048];

    struct pollfd fds[1];
    fds[0].fd = _fd;
    fds[0].events = POLLIN;

    while (!_should_exit) {
        int recv_len;
        int pollrc = poll(fds, 1, 1000);
        if (pollrc == 0 || !(fds[0].revents & POLLIN)) {
            continue;
        } else if (pollrc == -1) {
            std::cout << "read poll failure: " << std::endl;
        }
        // We enter here if (fds[0].revents & POLLIN) == true
        recv_len = static_cast<int>(read(_fd, buffer, sizeof(buffer)));
        if (recv_len < -1) {
            std::cout << "read failure: " << std::endl;
        }

        if (recv_len > static_cast<int>(sizeof(buffer)) || recv_len == 0) {
            continue;
        }



        set_new_datagram(buffer, recv_len);

        // Parse all mavlink messages in one data packet. Once exhausted, we'll exit while.
        while (parse_message()) {
            // _msg_q.append(_last_message);
            // std::cout << "Received message! \nid: " << _last_message.msgid << std::endl << std::endl;

            if (_last_message.msgid == MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST) {
            }

            switch (_last_message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    if (!_connected) {
                        std::cout << "Connected to System ID: " << int(_last_message.sysid) << std::endl;
                        _connected = true;
                        // TODO: connection timeout
                    }
                    break;

                case MAVLINK_MSG_ID_COMMAND_LONG:
                    handleCommandLong(_last_message);
                    break;

                case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
                    std::cout << SUCCESS_CONSOLE_TEXT << "Got MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST" << NORMAL_CONSOLE_TEXT << std::endl;
                    // TODO: do something....
                    break;

                case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
                    std::cout << SUCCESS_CONSOLE_TEXT << "Got MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ" << NORMAL_CONSOLE_TEXT << std::endl;
                    // TODO: do something....
                    break;
            }
        }
    }
}

void start_recv_thread()
{
    std::cout << "Starting receive thread" << std::endl;
    _recv_thread = new std::thread(receive);
}

void stop()
{
    _should_exit = true;

    if (_recv_thread) {
        _recv_thread->join();
        delete _recv_thread;
        _recv_thread = nullptr;
    }

    close(_fd);
}

void set_new_datagram(char* datagram, unsigned datagram_len)
{
    _datagram = datagram;
    _datagram_len = datagram_len;
}

bool parse_message()
{
    // Note that one datagram can contain multiple mavlink messages.
    for (unsigned i = 0; i < _datagram_len; ++i) {
        if (mavlink_parse_char(_channel, _datagram[i], &_last_message, &_status) == 1) {
            // Move the pointer to the datagram forward by the amount parsed.
            _datagram += (i + 1);
            // And decrease the length, so we don't overshoot in the next round.
            _datagram_len -= (i + 1);
            // We have parsed one message, let's return so it can be handled.
            return true;
        }
    }

    // No (more) messages, let's give up.
    _datagram = nullptr;
    _datagram_len = 0;
    return false;
}

bool send_message(const mavlink_message_t& message)
{
    std::lock_guard<std::mutex> lck (_send_mutex);

    if (_serial_node.empty()) {
        std::cout << "Dev Path unknown" << std::endl;
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

    int send_len;
    send_len = static_cast<int>(write(_fd, buffer, buffer_len));

    if (send_len != buffer_len) {
        std::cout << "write failure" << std::endl;
        return false;
    }

    return true;
}

void send_command_ack_message(uint16_t command)
{
    uint8_t result {};
    uint8_t progress {};
    int32_t result_param2 {};
    uint8_t target_system {};
    uint8_t target_component {};

    // Fill in the data
    result = MAV_RESULT_ACCEPTED;
    target_system = QGROUNDCONTROL_SYS_ID;

    mavlink_message_t message;
    mavlink_msg_command_ack_pack(
        AUTOPILOT_SYS_ID,
        MAV_COMP_ID_CAMERA,
        &message,
        // Messages specific
        command,
        result,
        progress,
        result_param2,
        target_system,
        target_component);

    send_message(message);
}

void send_camera_information_message()
{
    std::cout << "send_camera_information_message()" << std::endl;

    uint32_t time_boot_ms {};               //  ms                  Timestamp (time since system boot).
    uint8_t vendor_name[32] {};             //                      Name of the camera vendor
    uint8_t model_name[32]  {};             //                      Name of the camera model
    uint32_t firmware_version {};           //                      Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff)
    float focal_length {};                  //  mm                  Focal length
    float sensor_size_h {};                 //  mm                  Image sensor size horizontal
    float sensor_size_v {};                 //  mm                  Image sensor size vertical
    uint16_t resolution_h {};               //  pix                 Horizontal image resolution
    uint16_t resolution_v {};               //  pix                 Vertical image resolution
    uint8_t lens_id {};                     //                      Reserved for a lens ID
    uint32_t flags {};                      //  CAMERA_CAP_FLAGS    Bitmap of camera capability flags.
    uint16_t cam_definition_version {};     //                      Camera definition version (iteration)
    char cam_definition_uri[140] {};        //

    // Fill it in
    time_boot_ms = 1000*microsSinceEpoch();
    std::string vendor = "Watts Innovations";
    std::string model = "Example Camera";
    memcpy(vendor_name, vendor.c_str(), vendor.size());
    memcpy(model_name, model.c_str(), model.size());

    std::string url = "http://wattsinnovations.asuscomm.com:8000/WattsQGC/camera_def/watts_camera_definition_example.xml";
    memcpy(cam_definition_uri, url.c_str(), url.size());

    flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS;

    mavlink_message_t message;
    mavlink_msg_camera_information_pack(
        AUTOPILOT_SYS_ID,
        MAV_COMP_ID_CAMERA,
        &message,
        // Messages specific
        time_boot_ms,
        vendor_name,
        model_name,
        firmware_version,
        focal_length,
        sensor_size_h,
        sensor_size_v,
        resolution_h,
        resolution_v,
        lens_id,
        flags,
        cam_definition_version,
        cam_definition_uri);

    auto result = send_message(message);
}

void send_camera_settings_message()
{
    std::cout << "send_camera_settings_message()" << std::endl;

    uint32_t time_boot_ms {};   //  ms  Timestamp (time since system boot).
    // TODO: mutex it
    uint8_t mode_id {};         //      CAMERA_MODE     Camera mode -- CAMERA_MODE_IMAGE ,CAMERA_MODE_VIDEO, CAMERA_MODE_IMAGE_SURVEY
    float zoomLevel {};         //      Current zoom level (0.0 to 100.0, NaN if not known)
    float focusLevel {};        //      Current focus level (0.0 to 100.0, NaN if not known)

    // Fill it in
    time_boot_ms = 1000*microsSinceEpoch();
    // TODO: mutex it
    mode_id = 0;
    zoomLevel = NAN;
    focusLevel = NAN;

    mavlink_message_t message;
    mavlink_msg_camera_settings_pack(
        AUTOPILOT_SYS_ID,
        MAV_COMP_ID_CAMERA,
        &message,
        // Messages specific
        time_boot_ms,
        mode_id,
        zoomLevel,
        focusLevel);

    auto result = send_message(message);
}

void send_camera_capture_status_message()
{
    std::cout << "send_camera_capture_status_message()" << std::endl;

    uint32_t time_boot_ms {};           //  ms      Timestamp (time since system boot).
    uint8_t image_status {};            //          Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
    uint8_t video_status {};            //          Current status of video capturing (0: idle, 1: capture in progress)
    float image_interval {};            //  s       Image capture interval
    uint32_t recording_time_ms {};      //          ms  Time since recording started
    float available_capacity {};        //          MiB Available storage capacity.
    // TODO: mutex it
    int32_t image_count {};             //          Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).

    // Fill it in
    time_boot_ms = 1000*microsSinceEpoch();
    available_capacity = 4096;
    image_count = _image_count;

    mavlink_message_t message;
    mavlink_msg_camera_capture_status_pack(
        AUTOPILOT_SYS_ID,
        MAV_COMP_ID_CAMERA,
        &message,
        // Messages specific
        time_boot_ms,
        image_status,
        video_status,
        image_interval,
        recording_time_ms,
        available_capacity);

    auto result = send_message(message);
}

void send_storage_information_message()
{
    std::cout << "send_storage_information_message()" << std::endl;

    uint32_t time_boot_ms {};           //  ms      Timestamp (time since system boot).
    uint8_t storage_id {};              //          Storage ID (1 for first, 2 for second, etc.)
    uint8_t storage_count {};           //          Number of storage devices
    uint8_t status {};                  //          STORAGE_STATUS  Status of storage
    float total_capacity {};            //  MiB     Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
    float used_capacity {};             //  MiB     Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
    float available_capacity {};        //  MiB     Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
    float read_speed {};                //  MiB/s   Read speed.
    float write_speed {};               //  MiB/s   Write speed.

    // Fill it in
    time_boot_ms = 1000*microsSinceEpoch();
    storage_id = 1;
    storage_count = 1;
    status = STORAGE_STATUS_READY;
    total_capacity = 4096;
    used_capacity = 1024;
    available_capacity = 4096;

    mavlink_message_t message;
    mavlink_msg_storage_information_pack(
        AUTOPILOT_SYS_ID,
        MAV_COMP_ID_CAMERA,
        &message,
        // Messages specific
        time_boot_ms,
        storage_id,
        storage_count,
        status,
        total_capacity,
        used_capacity,
        available_capacity,
        read_speed,
        write_speed);

    auto result = send_message(message);
}

void send_camera_image_captured_message()
{
    // NOTE: this message is only used to display Camera Trigger Points in QGC (lat/lon)

    std::cout << "send_camera_image_captured_message()" << std::endl;

    uint32_t time_boot_ms {};               //  ms      Timestamp (time since system boot).
    uint64_t time_utc {};                   //  us      Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
    uint8_t camera_id {};                   //          Deprecated/unused. Component IDs are used to differentiate multiple cameras.
    int32_t lat {};                         //  degE7   Latitude where image was taken
    int32_t lon {};                         //  degE7   Longitude where capture was taken
    int32_t alt {};                         //  mm      Altitude (MSL) where image was taken
    int32_t relative_alt {};                //  mm      Altitude above ground
    float q[4] {};                          //          Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    int32_t image_index {};                 //          Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)
    int8_t capture_result {};               //          Boolean indicating success (1) or failure (0) while capturing this image.
    char file_url[205] {};                  //          URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.

    // Fill it in
    time_boot_ms = 1000*microsSinceEpoch();

    mavlink_message_t message;
    mavlink_msg_camera_image_captured_pack(
        AUTOPILOT_SYS_ID,
        MAV_COMP_ID_CAMERA,
        &message,
        // Messages specific
        time_boot_ms,
        time_utc,
        camera_id,
        lat,
        lon,
        alt,
        relative_alt,
        q,
        image_index,
        capture_result,
        file_url);

    auto result = send_message(message);
}

void handleCommandLong(const mavlink_message_t& message)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&message, &cmd);

    switch(cmd.command) {
        case MAV_CMD_REQUEST_CAMERA_INFORMATION:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_REQUEST_CAMERA_INFORMATION" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            send_camera_information_message();
            break;

        case MAV_CMD_REQUEST_CAMERA_SETTINGS:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_REQUEST_CAMERA_SETTINGS" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            send_camera_settings_message();
            break;

        case MAV_CMD_REQUEST_STORAGE_INFORMATION:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_REQUEST_STORAGE_INFORMATION" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            send_storage_information_message();
            break;

        case MAV_CMD_STORAGE_FORMAT:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_STORAGE_FORMAT" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            std::cout << ERROR_CONSOLE_TEXT << "TODO: format the storage" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            send_camera_capture_status_message();
            break;

        case MAV_CMD_SET_CAMERA_MODE:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_SET_CAMERA_MODE" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: set the mode
            std::cout << ERROR_CONSOLE_TEXT << "TODO: set the mode" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_SET_CAMERA_ZOOM:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_SET_CAMERA_ZOOM" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            send_camera_settings_message();
            // TODO: set the zoom
            std::cout << ERROR_CONSOLE_TEXT << "TODO: set the zoom" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_SET_CAMERA_FOCUS:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_SET_CAMERA_FOCUS" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: set the focus
            std::cout << ERROR_CONSOLE_TEXT << "Settings the focus" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_IMAGE_START_CAPTURE:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_IMAGE_START_CAPTURE" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: send back send_camera_image_captured_message() every time a photo is taken
            std::cout << ERROR_CONSOLE_TEXT << "Triggering photo capture" << NORMAL_CONSOLE_TEXT << std::endl;
            _image_count++;
            break;

        case MAV_CMD_IMAGE_STOP_CAPTURE:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_IMAGE_STOP_CAPTURE" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: stop capturing images
            std::cout << ERROR_CONSOLE_TEXT << "TODO: stop capturing images" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_VIDEO_START_CAPTURE:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_VIDEO_START_CAPTURE" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: start capturing video
            std::cout << ERROR_CONSOLE_TEXT << "TODO: start capturing video" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_VIDEO_STOP_CAPTURE:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_VIDEO_STOP_CAPTURE" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: stop capturing video
            std::cout << ERROR_CONSOLE_TEXT << "TODO: stop capturing video" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_VIDEO_START_STREAMING:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_VIDEO_START_STREAMING" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: start streaming video
            std::cout << ERROR_CONSOLE_TEXT << "TODO: start streaming video" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_VIDEO_STOP_STREAMING:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_VIDEO_STOP_STREAMING" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: stop streaming video
            std::cout << ERROR_CONSOLE_TEXT << "TODO: stop streaming video" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: send VIDEO_STREAM_INFORMATION message
            std::cout << ERROR_CONSOLE_TEXT << "TODO: send VIDEO_STREAM_INFORMATION message" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        case MAV_CMD_REQUEST_VIDEO_STREAM_STATUS:
            std::cout << TELEMETRY_CONSOLE_TEXT << "MAV_CMD_REQUEST_VIDEO_STREAM_STATUS" << NORMAL_CONSOLE_TEXT << std::endl;
            send_command_ack_message(cmd.command);
            // TODO: send VIDEO_STREAM_STATUS message
            std::cout << ERROR_CONSOLE_TEXT << "TODO: send VIDEO_STREAM_STATUS message" << NORMAL_CONSOLE_TEXT << std::endl;
            break;

        default:
            std::cout << ERROR_CONSOLE_TEXT << "Command " << cmd.command << " not supported" << NORMAL_CONSOLE_TEXT << std::endl;
            break;
    }
}