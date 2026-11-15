/************************************************************/
/*    NAME: Jeremy Wenger                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ContactUDP.cpp                                  */
/*    DATE: November 2026                                   */
/************************************************************/

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "ContactUDP.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ContactUDP::ContactUDP()
{
  m_port             = 9300;
  m_max_accuracy     = 30;
  m_min_course_speed = 0.4;
  m_stale_time       = 10;
  m_platform_type    = "ship";
  m_platform_length  = 4;
  m_platform_color   = "red";
  m_report_var       = "NODE_REPORT";

  m_socket    = -1;
  m_geo_ok    = false;
  m_datagrams = 0;
  m_rejected  = 0;
}

//---------------------------------------------------------
// Destructor

ContactUDP::~ContactUDP()
{
  if(m_socket >= 0)
    close(m_socket);
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ContactUDP::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ContactUDP::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//   Purpose: Drain the socket, post a report per fix, and flag contacts
//            that have gone quiet.

bool ContactUDP::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_socket < 0)
    openSocket();
  else
    readSocket();

  map<string, ContactFix>::iterator p;
  for(p=m_contacts.begin(); p!=m_contacts.end(); p++) {
    string warning = "No fix from " + p->first + " for " + doubleToStringX(m_stale_time,0) + " s";
    if((m_curr_time - p->second.rcv_time) > m_stale_time)
      reportRunWarning(warning);
    else
      retractRunWarning(warning);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool ContactUDP::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "port")
      handled = setIntOnString(m_port, value) && (m_port > 0) && (m_port < 65536);
    else if(param == "max_accuracy")
      handled = setNonNegDoubleOnString(m_max_accuracy, value);
    else if(param == "min_course_speed")
      handled = setNonNegDoubleOnString(m_min_course_speed, value);
    else if(param == "stale_time")
      handled = setPosDoubleOnString(m_stale_time, value);
    else if(param == "platform_type")
      handled = setNonWhiteVarOnString(m_platform_type, value);
    else if(param == "platform_length")
      handled = setPosDoubleOnString(m_platform_length, value);
    else if(param == "platform_color")
      handled = setNonWhiteVarOnString(m_platform_color, value);
    else if(param == "report_var")
      handled = setNonWhiteVarOnString(m_report_var, value);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // The datum turns LAT/LON into the local X,Y every other app works in
  double lat_origin = 0;
  double lon_origin = 0;
  if(m_MissionReader.GetValue("LatOrigin", lat_origin) &&
     m_MissionReader.GetValue("LongOrigin", lon_origin))
    m_geo_ok = m_geodesy.Initialise(lat_origin, lon_origin);
  if(!m_geo_ok)
    reportConfigWarning("LatOrigin/LongOrigin missing or invalid: no X,Y in the reports");

  openSocket();
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ContactUDP::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
}

//---------------------------------------------------------
// Procedure: openSocket()
//   Purpose: Bind a non-blocking UDP socket on every interface, so the
//            same config works over Tailscale, WiFi and localhost.

bool ContactUDP::openSocket()
{
  string warning = "Cannot listen on UDP port " + intToString(m_port);
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(fd < 0) {
    reportRunWarning(warning);
    return(false);
  }
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(static_cast<uint16_t>(m_port));
  if(::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    close(fd);
    reportRunWarning(warning);
    return(false);
  }
  retractRunWarning(warning);
  m_socket = fd;
  return(true);
}

//---------------------------------------------------------
// Procedure: readSocket()

void ContactUDP::readSocket()
{
  char buf[1024];
  // Bounded so a flood of datagrams cannot hold up the app
  for(int i=0; i<200; i++) {
    struct sockaddr_in from;
    socklen_t flen = sizeof(from);
    ssize_t n = recvfrom(m_socket, buf, sizeof(buf)-1, 0,
			 reinterpret_cast<struct sockaddr*>(&from), &flen);
    if(n <= 0)
      break;
    buf[n] = '\0';
    m_datagrams++;

    char ip[INET_ADDRSTRLEN] = "?";
    inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
    string source = string(ip) + ":" + intToString(ntohs(from.sin_port));

    // One fix per line; a sender may batch a few lines in one datagram
    vector<string> lines = parseString(string(buf), '\n');
    for(unsigned int j=0; j<lines.size(); j++) {
      string line = stripBlankEnds(lines[j]);
      if((line != "") && !handleFix(line, source))
	m_rejected++;
    }
  }
}

//---------------------------------------------------------
// Procedure: handleFix()
//   Returns: false if the line was rejected (m_last_reject says why)

bool ContactUDP::handleFix(const string& line, const string& source)
{
  string name;
  double lat = 0, lon = 0, spd = 0, hdg = -1, acc = -1, fix_time = 0;
  bool   got_lat = false, got_lon = false;

  vector<string> pairs = parseString(line, ',');
  for(unsigned int i=0; i<pairs.size(); i++) {
    string value = pairs[i];
    string key   = toupper(biteStringX(value, '='));
    if(key == "NAME")
      name = tolower(value);
    else if(!isNumber(value))
      continue;
    else if(key == "LAT")  { lat = atof(value.c_str()); got_lat = true; }
    else if(key == "LON")  { lon = atof(value.c_str()); got_lon = true; }
    else if(key == "SPD")  spd = atof(value.c_str());
    else if(key == "HDG")  hdg = atof(value.c_str());
    else if(key == "ACC")  acc = atof(value.c_str());
    else if(key == "TIME") fix_time = atof(value.c_str());
  }

  // A name becomes a MOOS variable suffix and a community name downstream
  bool name_ok = (name != "") && (name.length() <= 20);
  for(unsigned int i=0; i<name.length(); i++)
    name_ok = name_ok && (isalnum(name[i]) || (name[i] == '_'));
  if(!name_ok) {
    m_last_reject = "bad or missing NAME: " + line.substr(0, 60);
    return(false);
  }
  if(!got_lat || !got_lon || (fabs(lat) > 90) || (fabs(lon) > 180) ||
     ((lat == 0) && (lon == 0))) {
    m_last_reject = name + ": bad or missing LAT/LON";
    return(false);
  }
  if((m_max_accuracy > 0) && (acc > m_max_accuracy)) {
    m_last_reject = name + ": accuracy " + doubleToStringX(acc,0) + " m worse than " +
      doubleToStringX(m_max_accuracy,0) + " m";
    return(false);
  }

  ContactFix& fix = m_contacts[name];
  if(fix.count == 0)
    fix.first_time = m_curr_time;
  fix.lat = lat;
  fix.lon = lon;
  fix.spd = (spd > 0) ? spd : 0;
  fix.acc = acc;
  fix.source = source;
  fix.rcv_time = m_curr_time;
  fix.latency = (fix_time > 0) ? (MOOSLocalTime(false) - fix_time) : 0;
  fix.count++;

  // A phone has no course when it is not moving: keep the last good one
  // so the contact does not swing to north every time the skiff stops.
  fix.hdg_held = (hdg < 0) || (hdg >= 360) || (fix.spd < m_min_course_speed);
  if(!fix.hdg_held)
    fix.hdg = hdg;

  if(m_geo_ok)
    m_geodesy.LatLong2LocalGrid(lat, lon, fix.y, fix.x);

  postNodeReport(name, fix);
  return(true);
}

//---------------------------------------------------------
// Procedure: postNodeReport()
//      Note: TIME is this community's clock at reception, not the phone's,
//            so a phone with a wrong clock still makes a usable contact.

void ContactUDP::postNodeReport(const string& name, const ContactFix& fix)
{
  string report = "NAME=" + name;
  report += ",TYPE=" + m_platform_type;
  report += ",TIME=" + doubleToString(m_curr_time, 2);
  if(m_geo_ok) {
    report += ",X=" + doubleToString(fix.x, 2);
    report += ",Y=" + doubleToString(fix.y, 2);
  }
  report += ",LAT=" + doubleToString(fix.lat, 7);
  report += ",LON=" + doubleToString(fix.lon, 7);
  report += ",SPD=" + doubleToString(fix.spd, 2);
  report += ",HDG=" + doubleToString(fix.hdg, 1);
  report += ",DEP=0";
  report += ",LENGTH=" + doubleToStringX(m_platform_length, 1);
  report += ",MODE=PHONE";
  report += ",COLOR=" + m_platform_color;
  Notify(m_report_var, report);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ContactUDP::buildReport()
{
  m_msgs << "Listening on UDP port " << m_port
	 << ((m_socket >= 0) ? "" : "  (NOT OPEN)") << endl;
  m_msgs << "Datagrams: " << m_datagrams << "   rejected lines: " << m_rejected << endl;
  if(m_last_reject != "")
    m_msgs << "Last reject: " << m_last_reject << endl;
  m_msgs << endl;

  ACTable actab(9);
  actab << "Name | Fixes | Hz | Age | Acc(m) | Lat(s) | Spd | Hdg | From";
  actab.addHeaderLines();
  map<string, ContactFix>::iterator p;
  for(p=m_contacts.begin(); p!=m_contacts.end(); p++) {
    const ContactFix& fix = p->second;
    double span = fix.rcv_time - fix.first_time;
    string rate = (span > 1) ? doubleToString((fix.count - 1) / span, 1) : "-";
    string hdg  = doubleToString(fix.hdg, 0) + (fix.hdg_held ? " (held)" : "");
    actab << p->first << fix.count << rate
	  << doubleToString(m_curr_time - fix.rcv_time, 1)
	  << ((fix.acc >= 0) ? doubleToString(fix.acc, 1) : "-")
	  << doubleToString(fix.latency, 2)
	  << doubleToString(fix.spd, 2) << hdg << fix.source;
  }
  if(m_contacts.size() == 0)
    m_msgs << "No fixes received yet." << endl;
  else
    m_msgs << actab.getFormattedString();

  return(true);
}
