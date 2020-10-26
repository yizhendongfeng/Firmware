/**
 * @copyright (C) 2020 版权所有
 * @brief 收集差分GPS、姿态信息，通过串口转发给无人船
 * @author 张纪敏 <869159813@qq.com>
 * @version 1.0.0
 * @date 2020-10-21
 */

#include <px4_tasks.h>
#include <px4_posix.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_time.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <v2.0/posatt/mavlink.h>

#include <drivers/drv_hrt.h>
#include <termios.h>
#include <string.h>
/**
 * daemon management function.
 */
extern "C" __EXPORT int usv_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int usv_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);
/**
 * uart initial.
 */
static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int harpoon_task;				/**< Handle of daemon task / thread */

using matrix::wrap_2pi;
uint16_t
cm_uint16_from_m_float(float m)
{
    if (m < 0.0f) {
        return 0;

    } else if (m > 655.35f) {
        return 65535;
    }

    return (uint16_t)(m * 100.0f);
}

int usv_thread_main(int argc, char *argv[])
{
    PX4_INFO("[daemon] starting\n");
//    orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */

    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6 gps2口
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_fd = uart_init((char*)"/dev/ttyS6");
    if(false == uart_fd)
    {
        PX4_INFO("uart_read:%d",uart_fd);
    }
    if(false == set_uart_baudrate(uart_fd, 115200))
    {
        PX4_INFO("[YCM]set_uart_baudrate is failed\n");
    }

    uint8_t buf[256];

    //订阅gps数据
    int _gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    vehicle_gps_position_s gps = {};
    //订阅姿态数据
    int _att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    vehicle_attitude_s att = {};

    mavlink_posatt_t mavlink_posatt = {};

    thread_running = true;
    while (!thread_should_exit) {
        bool pos_updated = false;
        orb_check(_gps_sub, &pos_updated);
        if(pos_updated) {
            orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &gps);
            mavlink_posatt.time_usec = gps.timestamp;
            mavlink_posatt.fix_type = gps.fix_type;
            mavlink_posatt.lat = gps.lat;
            mavlink_posatt.lon = gps.lon;
            mavlink_posatt.alt = gps.alt;
            mavlink_posatt.eph = gps.hdop * 100;
            mavlink_posatt.epv = gps.vdop * 100;

            mavlink_posatt.vel = cm_uint16_from_m_float(gps.vel_m_s);
            mavlink_posatt.vx = gps.vel_n_m_s * 100;
            mavlink_posatt.vy = gps.vel_e_m_s * 100;
            mavlink_posatt.vz = gps.vel_d_m_s * 100;
            mavlink_posatt.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
            mavlink_posatt.satellites_visible = gps.satellites_used;


        }

        bool att_updated = false;
        orb_check(_att_sub, &att_updated);
        if (att_updated) {
            orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att);
            matrix::Eulerf euler = matrix::Quatf(att.q);
            mavlink_posatt.roll = euler.phi();
            mavlink_posatt.pitch = euler.theta();
            mavlink_posatt.yaw = wrap_2pi(euler.psi());
            mavlink_posatt.rollspeed = att.rollspeed;
            mavlink_posatt.pitchspeed = att.pitchspeed;
            mavlink_posatt.yawspeed = att.yawspeed;

            mavlink_message_t msg;
            mavlink_msg_posatt_encode(1, 1, &msg, &mavlink_posatt);
            int len = mavlink_msg_to_send_buffer(buf, &msg);
            write(uart_fd, buf, len);
//            int bytesSend = write(uart_fd, buf, len);
//            PX4_INFO("bytes:%d, type:%d, lat:%d, yaw:%.1f", bytesSend,
//                     mavlink_posatt.fix_type, mavlink_posatt.lat, (double)math::degrees(wrap_2pi(mavlink_posatt.yaw)));
        }
        usleep(50000);//50ms
    }
    PX4_INFO("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}

static void
usage(const char *reason)
{
    if (reason) {
        PX4_INFO("%s\n", reason);
    }

    PX4_INFO("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        PX4_INFO("failed to open port: %s", uart_name);
        return false;
    }
    if(fcntl(serial_fd, F_SETFL, FNDELAY) < 0)
        PX4_INFO("none block mode set failed!");
    return serial_fd;
}

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
    case 2400:   speed = B2400;   break;
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
        PX4_INFO("ERR: baudrate: %d\n", baud);
        return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
    return true;
}


/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int usv_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            PX4_INFO("daemon already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit = false;
        harpoon_task = px4_task_spawn_cmd("usv",
                                          SCHED_DEFAULT,
                                          SCHED_PRIORITY_DEFAULT,
                                          2000,
                                          usv_thread_main,
                                          (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            PX4_INFO("\trunning\n");

        } else {
            PX4_INFO("\tnot started\n");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}
