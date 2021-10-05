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

static constexpr uint8_t AUTOPILOT_SYS_ID = 1;
static constexpr uint8_t WINCH_COMP_ID = 7;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define SUCCESS_CONSOLE_TEXT "\033[32m" // Turn text on console blue

uint64_t absolute_time_ms();

void send_ready_messages();
bool setup_port();

void start_recv_thread();
void receive();
bool parse_message();
void set_new_datagram(char* datagram, unsigned datagram_len);

void stop();

// void send_command_ack_message(uint16_t command);
bool send_message(const mavlink_message_t& message);

void calculate_bitrate(uint16_t bytes, uint64_t time);

//  Variables
int _fd = -1;
std::string _serial_path = "/dev/ttyUSB0";

std::mutex _send_mutex;
std::thread* _recv_thread = nullptr;
std::queue<mavlink_message_t> _msg_q;

volatile bool _should_exit = false;
bool _connected = false;

uint8_t _channel = 0;
char* _datagram = nullptr;
unsigned _datagram_len = 0;

mavlink_message_t _last_message = {};
mavlink_status_t _status = {};

int32_t _image_count = 0;

uint64_t _last_received_heartbeat_time = 0;
uint64_t _last_heartbeat_time = 0;
uint64_t _last_telem_time = 0;

uint64_t _sent_message_count = 0;

// Bandwidth calculation
uint64_t _accumulated_bytes = 0;
uint64_t _avg_bitrate = 0;
uint64_t _last_bitrate_calc_time = 0;

int _mav_comp_id = 0;

// Constants
static constexpr int HEARTBEAT_CONNECTION_TIMEOUT_MS = 2000;

static constexpr int HEARTBEAT_FREQUENCY = 1; // Hz
static constexpr uint64_t HEARTBEAT_INTERVAL_MS = (1.0 / HEARTBEAT_FREQUENCY) * 1000; // ms

static constexpr int A2Z_TELEM_FREQUENCY = 50; // Hz
static constexpr uint64_t A2Z_TELEM_INTERVAL_MS = (1.0 / A2Z_TELEM_FREQUENCY) * 1000; // ms

void sig_handler(int signum)
{
    if (signum == SIGINT) {
        printf("\n\nSIGINT\n");
        stop();
    }
}

void stop()
{
    _should_exit = true;

    if (_recv_thread) {
        _recv_thread->join();
        delete _recv_thread;
        _recv_thread = nullptr;
    }

    printf("Sent total: %lu\n", _sent_message_count);
    printf("Average bitrate: %lu\n", _avg_bitrate);

    close(_fd);
}

int main(int argc, char* argv[])
{
    if (argc > 2) {

        _serial_path = std::string(argv[1]);

        _mav_comp_id = atoi(argv[2]);
        printf("Starting on %s as Component ID: %d\n", _serial_path.c_str(), _mav_comp_id);

    } else {
        printf("USB path and Component ID required!\n");
        return -1;
    }

    signal(SIGINT,sig_handler); // Register signal handler

	bool success = setup_port();

	if (!success) {
		printf("setup_port() failed!\n");
		return -1;

	} else {
		printf("setup_port() success!\n");
	}

    start_recv_thread();

    printf("Waiting for connection...\n");

    while (!_connected && !_should_exit) {
        usleep(500000);
    }

    // Start event loop
    while (!_should_exit) {

        uint64_t time_now = absolute_time_ms();

        if ((time_now - _last_received_heartbeat_time) > HEARTBEAT_CONNECTION_TIMEOUT_MS) {
            printf("Connection timeout!\n");
            stop();
            return -1;
        }

        send_ready_messages();
    }
}

void send_ready_messages()
{
    uint64_t time_now = absolute_time_ms();

    // Send heartbeat
    if ((time_now - _last_heartbeat_time) > HEARTBEAT_INTERVAL_MS) {
        mavlink_message_t message;

        mavlink_msg_heartbeat_pack(
            AUTOPILOT_SYS_ID,
            _mav_comp_id,
            &message,
            _mav_comp_id,
            MAV_AUTOPILOT_INVALID,
            0,
            0,
            0);

        // Send heartbeat
        printf("Sent messages: %lu\nAverage bitrate: %lu\x1b[A\r", _sent_message_count, _avg_bitrate);

        // printf("Average bitrate: %lu", _avg_bitrate);

        // printf("sending heartbeat...\n");

        send_message(message);
        _last_heartbeat_time = absolute_time_ms();
    }

    // Send telemetry
    if ((time_now - _last_telem_time) > A2Z_TELEM_INTERVAL_MS) {

        mavlink_message_t message;

        mavlink_msg_a2z_telemetry_pack(
            AUTOPILOT_SYS_ID,
            _mav_comp_id,
            &message,
            0,      // broadcast
            0,
            A2Z_STATE_ON_GROUND, // state
            0,
            20,     // payload_height
            3);     // payload_weight

        // std::cout << "Sending a2z telem message" << std::endl;
        send_message(message);
        _last_telem_time = absolute_time_ms();
    }
}

bool setup_port()
{
    // open() hangs on macOS or Linux devices(e.g. pocket beagle) unless you give it O_NONBLOCK
    _fd = open(_serial_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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

    const int bitrate = B115200;


    if (cfsetispeed(&tc, bitrate) != 0) {
        std::cout << "cfsetispeed failed: " << std::endl;
        close(_fd);
        return false;
    }

    if (cfsetospeed(&tc, bitrate) != 0) {
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

void start_recv_thread()
{
    std::cout << "Starting receive thread" << std::endl;
    _recv_thread = new std::thread(receive);
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

            switch (_last_message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:

                    _last_received_heartbeat_time = absolute_time_ms();

                    if (!_connected) {
                        std::cout << "Connected to System ID: " << int(_last_message.sysid) << std::endl;
                        _connected = true;
                    }

                    break;

                case MAVLINK_MSG_ID_COMMAND_LONG:
                    // handleCommandLong(_last_message);
                    break;
            }
        }
    }
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
    std::lock_guard<std::mutex> lck(_send_mutex);

    if (_serial_path.empty()) {
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

    // Message sent successfully

    // Increment the message count
    _sent_message_count++;

    uint64_t now = absolute_time_ms();

    calculate_bitrate(buffer_len, now);

    return true;
}

void calculate_bitrate(uint16_t bytes, uint64_t time)
{
    _accumulated_bytes += bytes;

    uint64_t dt_ms = time - _last_bitrate_calc_time;

    if (dt_ms > 500) {

        float bitrate = (_accumulated_bytes * 10.0) / (dt_ms / 1000.0); // 10 bits per symbol

        _avg_bitrate = (_avg_bitrate * 0.5) + (bitrate * 0.5);

        // Reset
        _accumulated_bytes = 0;
        _last_bitrate_calc_time = time;
    }
}

uint64_t absolute_time_ms()
{
    struct timeval tv;
    uint64_t millis = 0;
    gettimeofday(&tv, NULL);
    millis =  ((uint64_t)tv.tv_sec) * 1000 + (tv.tv_usec / 1000);
    return millis;
}