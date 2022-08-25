#ifndef ARDUINO_SERIAL_H
#define ARDUINO_SERIAL_H

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

namespace arduino_serial{

    /** \brief Custom serial class to read com ports on linux */
    class ArduinoSerial
    {
        public:
            /** \brief Contructor */
            ArduinoSerial();

            /** \brief Open serial port */
            bool open_port(const char *_portname);

            /** \brief Close serial port */
            void close_port();

            /** \brief Read from port, stores in internal char[256] buffer */
            bool read_port();

            // void write_port();

            int serial_port;
            char read_buf [256];
            int last_num_bytes;
    };
}

#endif