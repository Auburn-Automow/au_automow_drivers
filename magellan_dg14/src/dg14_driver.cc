// ROS header
#include <ros/ros.h>

// STL and stuff
#include <algorithm>
#include <iterator>
#include <time.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>

// Boost
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

// Serial Communication
#include <serial/serial.h>

// ROS msg headers
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <magellan_dg14/UTMFix.h>

// Diagnostic Msgs
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

// TF utilities
#include <tf/transform_datatypes.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace std;
using namespace magellan_dg14;
namespace po = boost::program_options;

string
trim_trailing(string str) {
    string whitespaces(" \t\f\v\n\r");
    size_t found;

    found = str.find_last_not_of(whitespaces);
    if (found != string::npos)
        str.erase(found+1);
    else
        str.clear();
        
    return str;
}

class Gps {
    public:
        double easting_origin;
        double northing_origin;
        
        explicit Gps() : node("~") {
            node.param("easting_offset", easting_origin, 641733.6623);
            node.param("northing_offset", northing_origin, 3606768.27);
            current_fix_type = 0;
            current_num_satellites = 0;
            current_differential_corrections_age = 0.0f;
        }

        // explicit Gps(string port, int buad = 115200) {
        // }

        bool Init(string port, int baud = 115200) {
            /* TODO: Make sure the serial port opened properly */
            s_.setPort(port);
            s_.setBaudrate(baud);
            s_.setTimeout(250);
            s_.open();
            utc_time = 0.0;
            testing = false;

            setupPublishers();
            
            setupDiagnostics();

            ROS_DEBUG("Finished Init");
            return true;
        }

        void setupPublishers() {
            utm_fix_pub = node.advertise<magellan_dg14::UTMFix>("utm_fix", 1);
            navsat_fix_pub = node.advertise<NavSatFix>("fix", 1);
            gps_odom_pub = node.advertise<Odometry>("odometry", 1);
            gps_timer = node.createTimer(ros::Duration(1.0/5.0),
                &Gps::publish_callback, this);
        }

        void updateGPSQuality(diagnostic_updater::DiagnosticStatusWrapper &s) {
            string fix_type_str = "Unknown";
            switch (current_fix_type) {
                case 0:
                    s.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                              "No GPS fix");
                    fix_type_str = "None";
                    break;
                case 1:
                    s.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                              "Stand alone GPS fix");
                    fix_type_str = "Stand-alone";
                    break;
                case 2:
                    s.summary(diagnostic_msgs::DiagnosticStatus::OK,
                              "Floating RTK GPS fix");
                    fix_type_str = "Floating RTK";
                    break;
                case 3:
                    s.summary(diagnostic_msgs::DiagnosticStatus::OK,
                              "Fixed RTK GPS fix");
                    fix_type_str = "Fixed RTK";
                    break;
                case 4:
                case 5:
                    s.summary(diagnostic_msgs::DiagnosticStatus::OK,
                              "Fixed RTK GPS fix plus SBAS");
                    fix_type_str = "Fixed RTK plus SBAS";
                    break;
                default:
                    s.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "Unknown GPS fix type %i", current_fix_type);
                    break;
            }
            s.add("Fix Type", fix_type_str);
            s.add("Fix Type ID", current_fix_type);
            s.add("Number of Satellites", current_num_satellites);
            s.add("Differential Corrections Age",
                  current_differential_corrections_age);
        }

        void setupDiagnostics() {
            // Set the hardware ID
            updater.setHardwareID("Magellan DG14");
            // Monitor the GPS quality
            updater.add("GPS Quality", this, &Gps::updateGPSQuality);
            // TODO: Add more diagnostic monitors
        }
        
        bool InitTesting() {
            testing = true;
            return testing;
        }
        
        void Stop() {
            s_.close();
        }
        
        void Cancel() {
            Stop();
        }
        
        void Step() {
            string line = s_.readline();
            process_data(line);
        }
        
        void process_data(string &unparsed_tokens) {
            ros::Time time = ros::Time::now();

            unparsed_tokens = trim_trailing(unparsed_tokens);
            vector<string> tokens;
            double utc_time_old = utc_time;
            size_t last_i = 0;
            for (size_t i = 0; i < unparsed_tokens.size(); i++) {
                if (unparsed_tokens[i] == ',') {
                    tokens.push_back(unparsed_tokens.substr(last_i, i-last_i));
                    last_i = i + 1;
                }
            }
            string last_with_checksum = unparsed_tokens.substr(last_i,
                unparsed_tokens.length() - 1);
            
            if (last_with_checksum.find('*') != string::npos) {
                if (last_with_checksum.find('*') == 0) {
                    tokens.push_back("");
                }
                else {
                    tokens.push_back(last_with_checksum.substr(0,
                        last_with_checksum.find('*') - 1));
                }
            }
            else {
                tokens.push_back(last_with_checksum);
            }
            
            if (tokens.size() < 2) {
                return;
            }
            
            nav_fix.header.stamp = time;
            
            if (tokens[0] == "$PASHR" && 
                tokens[1] == "POS") {
                process_data_pos(tokens);
            }
            else if (tokens[0] == "$PASHR" && 
                     tokens[1] == "UTM") {
                process_data_utm(tokens);
            }
            else if (tokens[0] == "$GPGST") {
                process_data_gst(tokens);
            }
            else {
                ROS_WARN("Received an incompatable or"
                         " incomplete message: %s %s",
                         tokens[0].c_str(), tokens[1].c_str());
            }
            // publish
            if (testing) {
                if (utc_time != utc_time_old)
                    cout << "Test: Publishing msg" << endl;
            }
        }
        
        void publish_callback(const ros::TimerEvent& e) {
            navsat_fix_pub.publish(nav_fix);
            updater.update();
        }
        
    private:
        
        void process_data_utm(vector<string> &tokens) {
            magellan_dg14::UTMFix utm_fix;
            utm_fix.header.stamp = ros::Time::now();
            utm_fix.header.frame_id = "gps_link";
            utm_fix.utc_time = strtod(tokens[2].c_str(), NULL);
            utm_fix.utm_zone = tokens[3];
            utm_fix.easting = strtod(tokens[4].c_str(), NULL);
            utm_fix.northing = strtod(tokens[5].c_str(), NULL);
            /* status msgs 
             * int16 FIX_TYPE_NONE = 0
             * int16 FIX_TYPE_RAW = 1
             * int16 FIX_TYPE_FLOAT = 2
             * int16 FIX_TYPE_FIXED = 3
             * int16 FIX_TYPE_FIXED_SBAS = 4
             * int16 FIX_BESTESTER = 5 # ? not sure about this one
             */
            int mode = atoi(tokens[6].c_str());
            utm_fix.fix_type = mode;
            utm_fix.num_satellites = atoi(tokens[7].c_str());
            utm_fix.hdop = strtod(tokens[8].c_str(), NULL);
            utm_fix.antenna_height = strtod(tokens[9].c_str(), NULL);
            utm_fix.geoidal_separation = strtod(tokens[11].c_str(), NULL);
            utm_fix.differential_corrections_age = atoi(tokens[13].c_str());
            utm_fix.differential_station_id = tokens[14];
            utm_fix.position_covariance = nav_fix.position_covariance;
            utm_fix_pub.publish(utm_fix);

            gps_odom.header.stamp = utm_fix.header.stamp;
            gps_odom.header.frame_id = "odom";
            gps_odom.pose.pose.position.x = utm_fix.easting - easting_origin;
            gps_odom.pose.pose.position.y = utm_fix.northing - northing_origin;
            gps_odom.pose.pose.position.z = 0;

            
            // Use ENU covariance to build XYZRPY covariance
            boost::array<double, 36> covariance = {{
              utm_fix.position_covariance[0],
              utm_fix.position_covariance[1],
              utm_fix.position_covariance[2],
              0, 0, 0,
              utm_fix.position_covariance[3],
              utm_fix.position_covariance[4],
              utm_fix.position_covariance[5],
              0, 0, 0,
              utm_fix.position_covariance[6],
              utm_fix.position_covariance[7],
              utm_fix.position_covariance[8],
              0, 0, 0,
              0, 0, 0, 99999, 0, 0,
              0, 0, 0, 0, 99999, 0,
              0, 0, 0, 0, 0, 99999 
            }};

            gps_odom.pose.covariance = covariance;
            gps_odom_pub.publish(gps_odom);

            current_fix_type = utm_fix.fix_type;
            current_num_satellites = utm_fix.num_satellites;
            current_differential_corrections_age =
                utm_fix.differential_corrections_age;
            updater.update();
        }
        
        void process_data_pos(vector<string> &tokens) {
            /* $ rosmsg show sensor_msgs/NavSatFix 
                 uint8 COVARIANCE_TYPE_UNKNOWN=0
                 uint8 COVARIANCE_TYPE_APPROXIMATED=1
                 uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
                 uint8 COVARIANCE_TYPE_KNOWN=3
                 Header header
                   uint32 seq
                   time stamp
                   string frame_id
                 sensor_msgs/NavSatStatus status
                   int8 STATUS_NO_FIX=-1
                   int8 STATUS_FIX=0
                   int8 STATUS_SBAS_FIX=1
                   int8 STATUS_GBAS_FIX=2
                   uint16 SERVICE_GPS=1
                   uint16 SERVICE_GLONASS=2
                   uint16 SERVICE_COMPASS=4
                   uint16 SERVICE_GALILEO=8
                   int8 status
                   uint16 service
                 float64 latitude
                 float64 longitude
                 float64 altitude
                 float64[9] position_covariance
                 uint8 position_covariance_type
            */
            int mode = atoi(tokens[2].c_str());
            if (mode == 0) {
                nav_fix.status.status = NavSatStatus::STATUS_FIX;
            }
            else if (mode == 1) {
                nav_fix.status.status = NavSatStatus::STATUS_SBAS_FIX;
            }
            else if (mode == 2) {
                nav_fix.status.status = NavSatStatus::STATUS_SBAS_FIX;
            }
            else if (mode == 3) {
                nav_fix.status.status = NavSatStatus::STATUS_SBAS_FIX;
            }
            else {
                nav_fix.status.status = -1;
            }
            // Only support GPS, no GLONASS or other systems
            nav_fix.status.service = NavSatStatus::SERVICE_GPS;
            
            utc_time = strtod(tokens[4].c_str(), NULL);
            
            nav_fix.latitude = strtod(tokens[5].c_str(), NULL);
            nav_fix.longitude = strtod(tokens[7].c_str(), NULL);
            nav_fix.altitude = strtod(tokens[9].c_str(), NULL);;
             // If southern hemisphere, it needs to be negative
            if (tokens[6] == "S") {
                nav_fix.latitude *= -1;
            }
             // If West of Prime meridian, it needs to be negative
            if (tokens[8] == "W") {
                nav_fix.longitude *= -1;
            }
            // Update the heading for the odometry using track
            gps_odom.pose.pose.orientation =
              tf::createQuaternionMsgFromYaw(strtod(tokens[11].c_str(),NULL));
            updater.update();
        }
        
        void process_data_gst(vector<string> &tokens) {
            utc_time = strtod(tokens[1].c_str(), NULL);
            nav_fix.position_covariance[0] =
                pow(strtod(tokens[6].c_str(),NULL),2);
            nav_fix.position_covariance[4] =
                pow(strtod(tokens[7].c_str(),NULL),2);
            nav_fix.position_covariance[8] =
                pow(strtod(tokens[8].c_str(),NULL),2);
            nav_fix.position_covariance_type =
                NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            updater.update();
        }
        
        ros::NodeHandle node;
        ros::NodeHandle privnode;
        ros::Publisher  utm_fix_pub;
        ros::Publisher  navsat_fix_pub;
        ros::Publisher  gps_odom_pub;
        serial::Serial  s_;
        ros::Timer      gps_timer;
        NavSatFix       nav_fix;
        Odometry        gps_odom;
        bool            testing;
        double          utc_time;
        int             current_fix_type;
        int             current_num_satellites;
        double          current_differential_corrections_age;
        diagnostic_updater::Updater updater;
};

int main (int argc, char *argv[]) {
    ros::init(argc, argv, "dg14_driver");
    
    Gps gps;
    
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help", "Displays this message")
        ("src", po::value<string>(), "Set the serial source, "
                                     "defaults to /dev/gps")
        ("baud", po::value<int>(), "Set the baud rate, defaults to 115200")
        ("test", po::value<string>(), "Sets a test")
    ;
    string src = "/dev/gps";
    string test = "";
    int baud = 115200;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }
    
    if (vm.count("test")) {
        test = vm["test"].as<string>();
    }
    
    if (test == "") {
        if (vm.count("src")) {
            src = vm["src"].as<string>();
        }

        if (vm.count("baud")) {
            baud = vm["baud"].as<int>();
        }
        
        if (ros::param::has("src")) {
            ros::param::get("src", src);
        }

        if (ros::param::has("baud")) {
            ros::param::get("baud", baud);
        }

        gps.Init(src, baud);
        while (ros::ok()) {
            ros::spinOnce();
            gps.Step();
        }
        gps.Stop();
    }
    else {
        gps.InitTesting();
        ifstream ifile;
        ifile.open(test.c_str(), ifstream::in);
        while (ifile.good()) {
            string line;
            getline(ifile, line);
            gps.process_data(line);
        }
    }
    
    return 0;
}
