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
#include <signal.h>

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

#include "joystick.hpp"

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
void start_joystick_thread();

void stop();
bool parse_message();
bool send_message(const mavlink_message_t& message);
void send_heartbeat();
void send_do_mount_control();

//  Variables
int _fd = -1;
std::string _serial_node = "/dev/ttyUSB0";

std::mutex _send_mutex;
std::thread* _recv_thread = nullptr;
std::thread* _joystick_thread = nullptr;

bool _should_exit = false;
bool _connected = false;

char* _datagram = nullptr;
unsigned _datagram_len = 0;

mavlink_message_t _last_message = {};
mavlink_status_t _status = {};

bool _joystick_connected = false;
int _axis0_value = 0;
int _axis1_value = 0;

static constexpr int AXIS0_MAX_VALUE = 28000;
static constexpr int AXIS0_MIN_VALUE = -28000;

static constexpr int AXIS1_MAX_VALUE = 22000;
static constexpr int AXIS1_MIN_VALUE = -24000;

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

    send_heartbeat();

    start_recv_thread();
    start_joystick_thread();

    static constexpr uint64_t HEARTBEAT_INTERVAL_US = 1000000;
    uint64_t last_heartbeat = 0;

    while (!_should_exit) {

        auto now = microsSinceEpoch();

        // Rate limit the heartbeat
        if (now > (last_heartbeat + HEARTBEAT_INTERVAL_US)) {
            send_heartbeat();
            last_heartbeat = now;
        }

        // Rate limit the joystick
        send_do_mount_control();

        usleep(100000); // 10hz
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

    const int baudrate = B57600;


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

void receive_thread_main()
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

            switch (_last_message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    if (!_connected) {
                        std::cout << "Connected to System ID: " << int(_last_message.sysid) << " Component ID: " << int(_last_message.compid) << std::endl;
                        _connected = true;
                        // TODO: connection timeout
                    }
                    break;

                case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
                    // std::cout << "Got MOUNT_ORIENTATION" << std::endl;
                    break;
                default:
                    // std::cout << "Got " << _last_message.msgid << std::endl;
                    break;
            }
        }
    }
}

void joystick_thread_main()
{
    Joystick joystick("/dev/input/js0", true); // create joystick in blocking mode

    _joystick_connected = true;

    while (!_should_exit) {

        JoystickEvent event;
        if (joystick.sample(&event)) {
            if (event.isButton()) {
                printf("Button %u is %s\n", event.number, event.value == 0 ? "up" : "down");

            } else if (event.isAxis()) {
                printf("Axis %u is at position %d\n", event.number, event.value);

                switch (event.number) {
                    case 0:
                        _axis0_value = event.value;
                        break;
                    case 1:
                        _axis1_value = event.value;
                        break;
                }
            }
        }

        usleep(10000); // 10ms == 100hz
    }
}

void start_recv_thread()
{
    std::cout << "Starting receive thread" << std::endl;
    _recv_thread = new std::thread(receive_thread_main);
}

void start_joystick_thread()
{
    std::cout << "Starting joystick thread" << std::endl;
    _joystick_thread = new std::thread(joystick_thread_main);
}

void stop()
{
    _should_exit = true;

    if (_recv_thread) {
        _recv_thread->join();
        delete _recv_thread;
        _recv_thread = nullptr;
    }

    if (_joystick_thread) {
        _joystick_thread->join();
        delete _joystick_thread;
        _joystick_thread = nullptr;
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
        if (mavlink_parse_char(0, _datagram[i], &_last_message, &_status) == 1) {
            // Move the pointer to the datagram forward by the amount parsed.
            _datagram += (i + 1);
            // And decrease the length, so we don't overshoot in the next round.
            _datagram_len -= (i + 1);
            // We have parsed one message, let's return so it can be handled.
            // std::cout << "got mavlink message" << std::endl;
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

void send_do_mount_control()
{
    float pitch = 0;
    float yaw = 0;

    mavlink_message_t message;
    mavlink_msg_command_long_pack(
        QGROUNDCONTROL_SYS_ID,
        0,
        &message,
        // PAYLOAD
        AUTOPILOT_SYS_ID,
        // MAV_COMP_ID_GIMBAL,
        0,
        MAV_CMD_DO_MOUNT_CONTROL,
        false,
        // Params 1 - 7
        pitch,
        0,
        yaw,
        0,
        0,
        0,
        MAV_MOUNT_MODE_MAVLINK_TARGETING);

    if (_joystick_connected) {
        printf("Sending MAV_CMD_DO_MOUNT_CONTROL\n");
        send_message(message);
    }
}

void send_heartbeat()
{
    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(
        QGROUNDCONTROL_SYS_ID,
        0,
        &message,
        0,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        0);

    if (_connected) {
        printf("sending heartbeat\n");
        send_message(message);
    }
}
