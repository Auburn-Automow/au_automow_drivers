#include "dg14/dg14.h"

#include <iostream>
#include <sstream>
#include <cstdio>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace dg14;

using std::string;
using std::stringstream;
using serial::Serial;
using serial::utils::SerialListener;

inline void defaultInfo(const string &msg) {
    std::cout << "DG14 Info: " << msg << std::endl;
}

void default_handler(std::string token) {
  std::cout << "default_handler got a: " << token << std::endl;
}

DG14::DG14 (string port):
    port_(""), serial_port_(NULL), serial_listener_(1),
    connected_(false)
    {
        this->port_ = port;
        this->info = defaultInfo;

        if (!this->port_.empty()) {
            this->connect();
        }
    }

DG14::~DG14() {
    this->disconnect();
}

void DG14::connect (string port) {
    if (this->connected_) {
        DG14_THROW(ConnectionException, "already connected");
    }

    if (!port.empty()) {
        this->port_ = port;
    }

    if (this->port_.empty()) {
        DG14_THROW(ConnectionException, "serial port name is empty");
    }

    this->disconnect();
    this->setupFilters_();

    this->serial_port_ = new Serial();
    this->serial_port_->setPort(this->port_);
    this->serial_port_->setBaudrate(9600);
    this->serial_port_->setParity(serial::parity_none);
    this->serial_port_->setStopbits(serial::stopbits_one);
    this->serial_port_->setBytesize(serial::eightbits);
    this->serial_port_->setTimeout(10);
    this->serial_port_->open();

    this->serial_listener_.setChunkSize(2);
    this->serial_listener_.startListening((*this->serial_port_));

    if (this->checkConnection())
    {
        this->connected_ = true;
        this->info("Connection established.");
    } else
        DG14_THROW(ConnectionException, "could not establish serial communication");
}

void DG14::disconnect () {
    this->connected_ = false;
    if (this->serial_listener_.isListening()) {
        this->serial_listener_.stopListening();
    }
    if (this->serial_port_ != NULL) {
        delete this->serial_port_;
        this->serial_port_ = NULL;
    }
}

bool DG14::isConnected () {
    return this->connected_;
}

bool DG14::checkConnection(void) {
    bool isConnected = false;
    serial::utils::BufferedFilterPtr ok_filt =
        this->serial_listener_.createBufferedFilter(SerialListener::contains("$PASHR,PRT"));

    this->serial_port_->write("$PASHQ,PRT\r\n");
    for (int i=0; i < 10; ++i)
    {
        if (!ok_filt->wait(50).empty()) {
            isConnected = true;
            break;
        }
    }
    return isConnected;
}

void DG14::setupFilters_() {
    this->serial_listener_.setDefaultHandler(default_handler);
}

