/************************************************************/
/*    NAME: Jeremy Wenger                                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BoatState.cpp                                   */
/*    DATE: November 2026                                   */
/************************************************************/

#include "BoatState.h"

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

using namespace std;

namespace
{
// The two files are small, machine-written JSON objects whose key names are
// unique across nesting levels, so a key lookup is enough; no JSON library.

// Position just past `"key":`, or npos.
size_t valuePos(const string& text, const string& key)
{
    size_t pos = text.find("\"" + key + "\"");
    if (pos == string::npos)
        return string::npos;
    pos = text.find(':', pos + key.size() + 2);
    if (pos == string::npos)
        return string::npos;
    pos = text.find_first_not_of(" \t\r\n", pos + 1);
    return pos;
}

bool jsonNumber(const string& text, const string& key, double& out)
{
    size_t pos = valuePos(text, key);
    if (pos == string::npos)
        return false;
    const char* start = text.c_str() + pos;
    char* end = nullptr;
    double v = strtod(start, &end);
    if (end == start)               // null, a string, or an object
        return false;
    out = v;
    return true;
}

bool jsonBool(const string& text, const string& key, bool& out)
{
    size_t pos = valuePos(text, key);
    if (pos == string::npos)
        return false;
    if (text.compare(pos, 4, "true") == 0)  { out = true;  return true; }
    if (text.compare(pos, 5, "false") == 0) { out = false; return true; }
    double v;
    if (jsonNumber(text, key, v))   { out = (v != 0); return true; }
    return false;
}

bool jsonString(const string& text, const string& key, string& out)
{
    size_t pos = valuePos(text, key);
    if (pos == string::npos || text[pos] != '"')
        return false;
    string s;
    for (size_t i = pos + 1; i < text.size() && text[i] != '"'; i++)
    {
        if (text[i] == '\\' && i + 1 < text.size())
            i++;
        s += text[i];
    }
    out = s;
    return true;
}

// Min and max of a JSON array of numbers; false if the array is absent or empty.
bool jsonArrayRange(const string& text, const string& key, double& lo, double& hi)
{
    size_t pos = valuePos(text, key);
    if (pos == string::npos || text[pos] != '[')
        return false;
    size_t close = text.find(']', pos);
    if (close == string::npos)
        return false;
    string body = text.substr(pos + 1, close - pos - 1);
    for (char& c : body)
        if (c == ',') c = ' ';
    istringstream iss(body);
    double v;
    bool any = false;
    while (iss >> v)
    {
        if (!any || v < lo) lo = v;
        if (!any || v > hi) hi = v;
        any = true;
    }
    return any;
}

// Whole file plus its age from the modification time.
bool slurp(const string& path, string& text, double& age)
{
    struct stat st;
    if (path.empty() || stat(path.c_str(), &st) != 0)
        return false;
    ifstream in(path);
    if (!in)
        return false;
    stringstream ss;
    ss << in.rdbuf();
    text = ss.str();
    age = difftime(time(nullptr), st.st_mtime);
    if (age < 0)
        age = 0;
    return !text.empty();
}
} // namespace

//---------------------------------------------------------
// Procedure: vcbStateName / vcbFaultName

string vcbStateName(int state)
{
    switch (state)
    {
    case 0: return "FAULT";
    case 1: return "IDLE";
    case 2: case 3: case 4: return "STARTING";
    case 5: return "RUN";
    default: return "UNKNOWN";
    }
}

string vcbFaultName(int fault)
{
    switch (fault)
    {
    case 0: return "none";
    case 1: return "PWM lost or invalid";
    case 2: return "LSC already closed at start";
    case 3: return "LSC did not close";
    case 4: return "HSC already closed at start";
    case 5: return "HSC did not close";
    case 6: return "bus under 40 V after closing";
    case 7: return "bus under 35 V in RUN";
    case 8: return "contactor opened in RUN (TSMS), latched";
    case 9: return "software stop, latched";
    default: return "fault " + to_string(fault);
    }
}

//---------------------------------------------------------
// Procedure: readBmsFile

bool readBmsFile(const string& path, double stale_sec, BmsState& s)
{
    s = BmsState();
    string text;
    if (!slurp(path, text, s.age))
        return false;
    s.fresh = (s.age <= stale_sec);
    jsonBool(text, "online", s.online);

    double v = 0;
    if (jsonNumber(text, "voltage", v)) s.voltage = v;
    if (jsonNumber(text, "current", v)) s.current = (v == 0) ? 0.0 : -v;
    if (jsonNumber(text, "soc", v))     s.soc = v;
    if (jsonNumber(text, "temp", v))    s.temp = v;
    s.power = s.voltage * s.current;
    jsonArrayRange(text, "cells", s.cell_min, s.cell_max);
    return true;
}

//---------------------------------------------------------
// Procedure: readVcbFile

bool readVcbFile(const string& path, double stale_sec, VcbState& s)
{
    s = VcbState();
    string text;
    if (!slurp(path, text, s.age))
        return false;
    s.fresh = (s.age <= stale_sec);
    jsonBool(text, "online", s.online);

    // thruster_state is a number on firmware 0.3.x and a name on older builds
    double v = 0;
    string name;
    if (jsonNumber(text, "thruster_state", v))
        s.state = static_cast<int>(v);
    else if (jsonString(text, "thruster_state", name))
        s.state = (name == "RUN") ? 5 : (name == "IDLE") ? 1 : (name == "FAULT") ? 0 : 3;

    if (jsonNumber(text, "fault_code", v)) s.fault = static_cast<int>(v);
    jsonBool(text, "armed", s.armed);
    jsonBool(text, "tsms_24v_present", s.tsms);
    bool lsc = false, hsc = false;
    jsonBool(text, "lsc_closed", lsc);
    jsonBool(text, "hsc_closed", hsc);
    s.hv_on = lsc && hsc;
    if (jsonNumber(text, "bus_mv", v)) s.bus_volts = v / 1000.0;

    // ch1 = right motor, ch2 = left (measured on the boat 2026-10-25)
    if (jsonNumber(text, "ch1_applied_permille", v)) s.cmd_right = v / 10.0;
    if (jsonNumber(text, "ch2_applied_permille", v)) s.cmd_left = v / 10.0;
    bool v1 = false, v2 = false;
    jsonBool(text, "ch1_valid", v1);
    jsonBool(text, "ch2_valid", v2);
    s.pwm_valid = v1 && v2;
    jsonString(text, "last_event", s.last_event);
    return true;
}
