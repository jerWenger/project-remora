/************************************************************/
/*    NAME: Jeremy Wenger                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ContactUDP.h                                    */
/*    DATE: November 2026                                   */
/************************************************************/

// Position fixes from a phone (or anything else with a GPS) arrive as one
// UDP datagram per fix and leave as NODE_REPORT, so the sender shows up in
// pMarineViewer and, through uFldNodeComms, as a contact on the vehicles.
//
// Datagram: one line of comma-separated KEY=value pairs, any order:
//   NAME=skiff,LAT=42.358431,LON=-71.087512,SPD=1.42,HDG=268.5,TIME=1794772478.31,ACC=4.1
// NAME, LAT and LON are required. SPD in m/s, HDG in degrees true (leave it
// out or send -1 when the device has no course, e.g. standing still), TIME is
// the sender's unix time of the fix, ACC its horizontal accuracy in metres.

#ifndef ContactUDP_HEADER
#define ContactUDP_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <map>
#include <string>

struct ContactFix
{
  double lat = 0;
  double lon = 0;
  double x = 0;
  double y = 0;
  double spd = 0;
  double hdg = 0;          // last valid course, held while the sender has none
  double acc = -1;         // m, -1 = not sent
  double latency = 0;      // receive time - sender's fix time, s (clock skew included)
  double rcv_time = 0;     // MOOSTime of the last fix
  double first_time = 0;
  unsigned int count = 0;
  bool   hdg_held = false; // the last fix carried no course
  std::string source;      // ip:port it came from
};

class ContactUDP : public AppCastingMOOSApp
{
 public:
   ContactUDP();
   ~ContactUDP();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool openSocket();
   void readSocket();
   bool handleFix(const std::string& line, const std::string& source);
   void postNodeReport(const std::string& name, const ContactFix& fix);

 private: // Configuration variables
   int         m_port;
   double      m_max_accuracy;     // m; worse fixes are dropped (0 = keep all)
   double      m_min_course_speed; // m/s; below this the reported course is not trusted
   double      m_stale_time;       // s without a fix before a contact is reported stale
   std::string m_platform_type;
   double      m_platform_length;
   std::string m_platform_color;
   std::string m_report_var;

 private: // State variables
   int          m_socket;
   CMOOSGeodesy m_geodesy;
   bool         m_geo_ok;
   std::map<std::string, ContactFix> m_contacts;
   unsigned int m_datagrams;
   unsigned int m_rejected;
   std::string  m_last_reject;
};

#endif
