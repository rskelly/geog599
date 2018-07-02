#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep
#include <stdlib.h>    // exit function
#include <inttypes.h>  // uint8_t, etc
#include <linux/i2c-dev.h> // I2C bus definitions
#include <signal.h>
#include <math.h>

#include <asio.hpp>

#include "encoder.hpp"

using namespace uav::sensor;
using asio::ip::udp;

bool __quit = false;

void _run(Encoder* enc, int fd, double* value, bool* running) {
	int max = 0;
	int port = 13;
	asio::io_service svc;
	udp::socket sock(svc, udp::endpoint(udp::v4(), port));

	while(*running) {

		udp::endpoint remote;
		asio::error_code err;
		std::vector<char> recv(128);
		sock.receive_from(asio::buffer(recv), remote, 0, err);

		if(err && err != asio::error::message_size)
			throw std::runtime_error(err.message());

                printf("%d\n", recv[0]);

		std::vector<double> send(1);

		while(*running) {
			if(__quit) {
				enc->stop();
				break;
			}
			// Read the encoder.
			int val = enc->readValue();
			// If the value is larger than the configed max, increase the max.
			if(val > max) max = val;
			// Compute the output value.
			*value = (double) val / max * 360.0;
                        printf("%f\n", *value);

			send[0] = *value;
			sock.send_to(asio::buffer(send), remote, 0, err);

			std::this_thread::yield(); //sleep_for(std::chrono::duration<std::milli>(1));
		}
	}
}


void sigintHandler(int p) {
	__quit = true;
}

void Encoder::configure(int addr, int mode) {
  char writeBuf[3];
  char readBuf[2];
  addr = 0b10000000 | (addr << 4) | mode;
  writeBuf[0] = 0b00000001;
  writeBuf[1] = addr;
  writeBuf[2] = 0b11100000;
  if(write(m_fd, writeBuf, 3) != 3) {
    perror("Write");
    exit(-1);
  }
  if(mode) {
    do {
      if(read(m_fd, writeBuf, 2) != 2) {
          perror("Read");
          exit(-1);
      }
    } while((writeBuf[0] & 0x80) == 0);
  } else {
    sleep(1);
  }
  readBuf[0] = 0;
  if(write(m_fd, readBuf, 1) != 1) {
    perror("Register select");
    exit(-1);
  }
}

int Encoder::readValue() {
  char readBuf[2];
  if(read(m_fd, readBuf, 2) != 2) {
      perror("Read conversion");
      exit(-1);
    }
    return readBuf[0] << 8 | readBuf[1];
}

Encoder::Encoder() :
		m_running(false),
		m_fd(0),
		m_value(std::numeric_limits<double>::quiet_NaN()){
	// Configure the interrupt handler.
	signal(SIGINT, sigintHandler);
}

void Encoder::start() {
	if(!m_running) {
		// Open the device with fd as the handle.
		if((m_fd = open("/dev/i2c-1", O_RDWR)) < 0) {
			printf("Error: Couldn't open device. %d\n", m_fd);
			exit(1);
		}
		// Connect to the device.
		if(ioctl(m_fd, I2C_SLAVE, asd_addr) < 0) {
			printf("Error: Couldn't find device at address %d\n", asd_addr);
			exit(1);
		}
		// Configure to read the angle continuously.
		configure(100, 0);

		m_running = true;
		//m_thread = std::thread(_run, this, m_fd, &m_value, &m_running);
		_run(this, m_fd, &m_value, &m_running);
	}
}

void Encoder::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
		close(m_fd);
	}
}


int main(int argc, char** argv) {

	Encoder enc;
	enc.start();

}
