/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Harpoon.cpp
 * Harpoon controller
 * 根据遥控器信号，控制鱼叉伸出与收入
 * @author 张纪敏 <869159813@qq.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <uORB/topics/manual_control_setpoint.h>

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <float.h>
#include <string.h>
#include <termios.h>

#define UP 1
#define UPSTOP 11//可以起飞
#define DOWN 2
#define DOWNSTOP 22//可以降落
/**
 * daemon management function.
 */
extern "C" __EXPORT int harpoon_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int harpoon_thread_main(int argc, char *argv[]);

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
int harpoon_thread_main(int argc, char *argv[])
{
    warnx("[daemon] starting\n");
    orb_advert_t	_mavlink_log_pub{nullptr};	/**< the uORB advert to send messages over mavlink */
    int uart_fd= -1;
    char harpoon_cmd = -1;   //1:起飞，2:降落
    unsigned char harpoon_status = -1;//0:向上运动，1:已收回可以起飞; 2:向下运动，3:已放下可以降落，
    char byteReaded = -1;
    //订阅遥控器信号
    int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    manual_control_setpoint_s manual_sp = {};


    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    uart_fd = uart_init((char*)"/dev/ttyS6");

    if(false == uart_fd)
    {
        PX4_INFO("uart_read:%d",uart_fd);
    }
    if(false == set_uart_baudrate(uart_fd,2400))
    {
        PX4_INFO("[YCM]set_uart_baudrate is failed\n");
    }
    thread_running = true;
    while (!thread_should_exit) {
        int readSize = read(uart_fd, &byteReaded, 1);
        PX4_INFO("readSize:%d,harpoon_status:%d",readSize,byteReaded);
        if(readSize)
        {
            if(harpoon_status != byteReaded)
            {
                harpoon_status = byteReaded;
                if(harpoon_status == UP){
                    mavlink_log_info(&_mavlink_log_pub, "harpoon retracting...");
                }
                if(harpoon_status == UPSTOP){
                    mavlink_log_info(&_mavlink_log_pub, "harpoon retracted!");
                }
                else if(harpoon_status == DOWN){
                    mavlink_log_info(&_mavlink_log_pub, "harpoon stretching out...");
                }
                else if(harpoon_status == DOWNSTOP){
                    mavlink_log_info(&_mavlink_log_pub, "harpoon stretched out!");
                }
            }
        }

        bool manual_updated = false;
        orb_check(manual_sub, &manual_updated);
        if(manual_updated)
        {
            PX4_INFO("manual_sp.gear_switch:%d,manual_sp.aux2:%.1f, harpoon_status:%d",manual_sp.gear_switch, (double)manual_sp.aux2, harpoon_status);
            orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual_sp);
            //当收到收起命令且鱼叉状态不为已收起则收起鱼叉
            if(manual_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && harpoon_status != UPSTOP)
            {
                harpoon_cmd = UP;
                write(uart_fd, &harpoon_cmd, 1);
            }
            //当收到伸出命令且鱼叉状态不为已伸出则伸出鱼叉
            else if(manual_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF && harpoon_status != DOWNSTOP)
            {
                harpoon_cmd = DOWN;
                write(uart_fd, &harpoon_cmd, 1);
            }
        }
        usleep(50000);//50ms
    }
    warnx("[daemon] exiting.\n");

    thread_running = false;

    return 0;
}

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
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
        warnx("ERR: baudrate: %d\n", baud);
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
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
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
int harpoon_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("daemon already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit = false;
        harpoon_task = px4_task_spawn_cmd("harpoon",
                                          SCHED_DEFAULT,
                                          SCHED_PRIORITY_DEFAULT,
                                          2000,
                                          harpoon_thread_main,
                                          (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}
