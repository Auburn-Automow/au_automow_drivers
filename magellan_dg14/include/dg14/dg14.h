#ifndef DG14_DG14_H
#define DG14_DG14_H

#include <string>
#include <sstream>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#define DG14_THROW(exceptionClass, message) throw \
    exceptionClass(__FILE__,__LINE__,(message))

namespace dg14 {

    enum ports {kA, kB, kC};
    enum bauds {k300, k600, k1200, k2400, k4800,
                k9600, k19200, k38400, k56800, k115200};

    typedef boost::function<void(const std::string&)> LoggingCallback;

    class DG14 {
    public:
        DG14 (std::string port = "");
        ~DG14 (void);
        void connect(std::string port = "");
        void disconnect (void);
        bool isConnected (void);

        LoggingCallback debug;
        LoggingCallback info;
        LoggingCallback warn;

    private:
        std::string port_;
        serial::Serial * serial_port_;
        serial::utils::SerialListener serial_listener_;

        bool checkConnection(void);
        void setupFilters_(void);
        serial::utils::BufferedFilterPtr test_filt_;

        bool connected_;
    };

      /*!
       * This exceptions occurs during connection.
       */
      class ConnectionException : public std::exception {
        std::string file_;
        int line_;
        const char* e_what_;
      public:
        ConnectionException (std::string file, int line, const char * description)
        : file_(file), line_(line), e_what_ (description) {}
        virtual ~ConnectionException() throw() {}

        virtual const char* what () const throw () {
          std::stringstream ss;
          ss << "Connection Exception: " << e_what_;
          ss << ", file " << file_ << ", line " << line_ << ".";
          return ss.str ().c_str ();
        }
      };
}

#endif
