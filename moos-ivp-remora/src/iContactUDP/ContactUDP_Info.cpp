/****************************************************************/
/*   NAME: Jeremy Wenger                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: ContactUDP_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ContactUDP_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  iContactUDP turns position fixes that arrive as UDP datagrams ");
  blk("  into NODE_REPORT, so a phone (or anything with a GPS and an   ");
  blk("  IP link) appears in pMarineViewer and, via uFldNodeComms, as  ");
  blk("  a contact on the vehicles. One line of KEY=value pairs per    ");
  blk("  fix: NAME, LAT, LON required; SPD, HDG, TIME, ACC optional.   ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iContactUDP file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iContactUDP with the given process name         ");
  blk("      rather than iContactUDP.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iContactUDP.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iContactUDP Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iContactUDP                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  port             = 9300   // UDP port, all interfaces         ");
  blk("  max_accuracy     = 30     // m; drop worse fixes (0 = keep)   ");
  blk("  min_course_speed = 0.4    // m/s; slower = hold last heading  ");
  blk("  stale_time       = 10     // s without a fix -> run warning   ");
  blk("  platform_type    = ship                                       ");
  blk("  platform_length  = 4      // m                                ");
  blk("  platform_color   = red                                        ");
  blk("  report_var       = NODE_REPORT                                ");
  blk("}                                                               ");
  blk("                                                                ");
  blk("Datagram, one line per fix (HDG -1 or absent = no course):      ");
  blk("  NAME=skiff,LAT=42.358431,LON=-71.087512,SPD=1.42,HDG=268.5,   ");
  blk("  TIME=1794772478.31,ACC=4.1                                    ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iContactUDP INTERFACE                                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  APPCAST_REQ                                                   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT = NAME=skiff,TYPE=ship,TIME=..,X=..,Y=..,LAT=..,  ");
  blk("                LON=..,SPD=..,HDG=..,DEP=0,LENGTH=4,MODE=PHONE, ");
  blk("                COLOR=red   (variable name set by report_var)   ");
  blk("  APPCAST                                                       ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iContactUDP", "gpl");
  exit(0);
}

