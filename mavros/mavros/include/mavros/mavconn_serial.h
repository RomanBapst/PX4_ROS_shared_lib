/**
 * @brief MAVConn Serial link class
 * @file mavconn_serial.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#pragma once

#include <list>
#include <atomic>
#include <boost/asio.hpp>
#include <mavros/mavconn_interface.h>
#include <mavros/mavconn_msgbuffer.h>

namespace mavconn {

/**
 * @brief Serial interface
 */
class MAVConnSerial : public MAVConnInterface {
public:
	/**
	 * Open and run serial link.
	 *
	 * @param[in] device    TTY device path
	 * @param[in] baudrate  serial baudrate
	 */
	MAVConnSerial(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string device = "/dev/ttyACM0", unsigned baudrate = 57600);
	~MAVConnSerial();

	void close();

	using MAVConnInterface::send_message;
	void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);
	void send_bytes(const uint8_t *bytes, size_t length);

	inline mavlink_status_t get_status() { return *mavlink_get_channel_status(channel); };
	inline bool is_open() { return serial_dev.is_open(); };

private:
	boost::asio::io_service io_service;
	std::thread io_thread;
	boost::asio::serial_port serial_dev;

	std::atomic<bool> tx_in_progress;
	std::list<MsgBuffer*> tx_q;
	uint8_t rx_buf[MsgBuffer::MAX_SIZE];
	std::recursive_mutex mutex;

	void do_read();
	void async_read_end(boost::system::error_code, size_t bytes_transferred);
	void do_write(bool check_tx_state);
	void async_write_end(boost::system::error_code, size_t bytes_transferred);
};

}; // namespace mavconn

