// Name: LandSimpleLambertSolverMFD
// Author: JitteryJet
// Version: V01
// kOS Version: 1.3.2.0
// KSP Version: 1.12.4
// Description:
//    Multi-function Display (MFD) for the LandSimpleLambertSolver program.
//
// Notes:
//    - Coded using Delegates.
//    - The screen is cleared and terminal size are set in the caller, it does not
//      make any assumptions about how the caller might want the terminal set.
//    - The screen can be refreshed as little or as often as possible.
//      The refresh can be done by a trigger, but keep it mind it will use up some of the
//      physics tick.
//    -
//
// Todo:
//    -
//
// Update History:
//    13/11/2022 V01  - Created.
//                    - 
//
// MFD Label Abbreviations:
//  Function      MFD Function Name.
//  Ship Name     Name of this ship.
//  VSpd          Ship vertical speed (m/s).
//  ASL           Ship altitude above sea level (km).
//  AGL           Ship altitude above ground level (km). 
//	Ap            Orbit Apoapsis (km).
//	Pe	          Orbit Periapsis (km).
//  Inc           Orbit Inclination (degrees).
//  Ecc           Orbit Eccentricity.
//	Stat	        Flight Status.
//  SType         Search Type.
//  Steps         Number of search steps.
//  TLat          Latitude of target (degrees).
//  TLng          Longitude of target (degrees).
//  DHt           Descent Height (km).
//  LHt           Landing height (m).
//  LSpd          Landing speed (m/s).
//	BnIn	        Maneuver Burn In Time (s).
//  BnDu	        Maneuver Burn Duration (s).
//	BnDv	        Maneuver Burn Delta-v (m/s).
//  DTime         Departure time being searched.
//  TTime         Transfer time being searched.
//  FTTme         Final transfer time after search.
//  FDTme         Final departure time after search.
//  FSMA          Final Transfer Semi-major Axis after search (km).
//  FTDv          Final transfer delta-v after search.
//
@lazyglobal off.
//
// Expose the delegates.
global MFDFunctions to lexicon
  (
    "DisplayLabels",DisplayLabels@,
    "DisplayRefresh",DisplayRefresh@,
    "DisplayDiagnostic",DisplayDiagnostic@,
    "DisplayManeuver",DisplayManeuver@,
    "DisplayFlightStatus",DisplayFlightStatus@,
    "DisplayError",DisplayError@,
    "DisplaySearchResults",DisplaySearchResults@
  ).

local Col1Col to 7.
local Col2Col to 38.
local ColSize is 18.
local LineSize is 56.
local VeryBigNumber to 3.402823E+38.

local ApLine to 0.
local PeLine to 0.
local EccLine to 0.
local IncLine to 0.
local NameLine to 0.
local STypeLine to 0.
local StepsLine to 0.
local DTimeLine to 0.
local TTimeLine to 0.
local FDTmeLine to 0.
local FSMALine to 0.
local FTTmeLine to 0.
local FTDvLine to 0.
local StatLine to 0.
local Diag1Line to 0.
local Diag2Line to 0.
local BnInLine to 0.
local BnDuLine to 0.
local BnDvLine to 0.
local Error1Line to 0.
local VSpdLine to 0.
local ASLLine to 0.
local AGLLine to 0.
local TLatLine to 0.
local TLngLine to 0.
local DHtLine to 0.
local LHtLine to 0.
local LSpdLine to 0.

local function DisplayLabels
  {
// Display the labels, headings and any data that does not change.
// Calculate the line numbers for each datum.
// Notes:
//    - The two blank lines at the top of the screen allow for the
//      "Program ended" message line and the following cursor line.
//
    parameter MyShipName.
    parameter SType.
    parameter Steps.
    parameter TLat.
    parameter TLng.
    parameter DHt.
    parameter LHt.
    parameter LSpd.
    
    local line to 0.

//         -123456789-123456789-123456789-123456789-123456789-12345
//         XXXXXX XXXXXXXXXXXXXXXXXX      XXXXXX XXXXXXXXXXXXXXXXXX
    print "                                                        " at (0,line).
    set line to line+1.
    print "                                                        " at (0,line).
    set line to line+1.
    print "Function: Land Using Simple Lambert Solver              " at (0,line).
    set line to line+1.
    print "Ship Name:                                              " at (0,line).
    set NameLine to line. set line to line+1.
    print "----------SHIP-----------      --------AUTOPILOT--------" at (0,line).
    set line to line+1.
    print "VSpd:                          Stat:                    " at (0,line).
    set VSpdLine to line. set StatLine to line. set line to line+1.
    print "ASL:                           SType:                   " at (0,line).
    set ASLLine to line. set STypeLine to line. set line to line+1.
    print "AGL:                           Steps:                   " at (0,line).
    set AGLLine to line. set StepsLine to line. set line to line+1.
    print "                               TLat:                    " at (0,line).
    set TLatLine to line. set line to line+1.
    print "                               TLng:                    " at (0,line).
    set TLngLine to line. set line to line+1.
    print "                               DHt:                     " at (0,line).
    set DHtLine to line. set line to line+1.
    print "                               LHt:                     " at (0,line).
    set LHtLine to line. set line to line+1.
    print "                               LSpd:                    " at (0,line).
    set LSpdLine to line. set line to line+1.
    print "----------ORBIT----------      ---------SEARCH----------" at (0,line).
    set line to line+1.
    print "Ap:                            DTime:                   " at (0,line).
    set ApLine to line. set DTimeLine to line. set line to line+1.
    print "Pe:                            TTime:                   " at (0,line).
    set PeLine to line. set TTimeLine to line. set line to line+1.
    print "Inc:                                                    " at (0,line).
    set IncLine to line. set line to line+1.
    print "Ecc:                           -----SEARCH RESULTS------" at (0,line).
    set EccLine to line. set line to line+1.
    print "-----ESTIMATED BURN------      FDTme:                   " at (0,line).
    set FDTmeline to line. set line to line+1.
    print "BnIn:                          FTTme:                   " at (0,line).
    set BnInLine to line. set FTTmeLine to line. set line to line+1.
    print "BnDu:                          FSMA:                    " at (0,line).
    set BnDuLine to line. set FSMALine to line. set line to line+1.
    print "BnDv:                          FTDv:                    " at (0,line).
    set BnDvLine to line. set FTDvLine to line. set line to line+1.
    print "--------ERROR MSG--------                               " at (0,line).
    set line to line+1.
    print "                                                        " at (0,line).
    set Error1Line to line. set line to line+1.
    print "-------DIAGNOSTICS-------                               " at (0,line).
    set line to line+1.
    print "                                                        " at (0,line).
    set Diag1Line to line. set line to line+1.
    print "                                                        " at (0,line).
    set Diag2Line to line. set line to line+1.

    print MyShipName at (11,Nameline).
    print SType at (Col2Col,STypeLine).
    print MFDVal(round(Steps,0)) at (Col2Col,StepsLine).
    print MFDVal(round(TLat,6)+char(176)) at (Col2Col,TLatLine).
    print MFDVal(round(TLng,6)+char(176)) at (Col2Col,TLngLine).
    print MFDVal(round(DHt/1000,3)+" km") at (Col2Col,DHtLine).
    print MFDVal(round(LHt,1)+" m") at (Col2Col,LHtLine).
    print MFDVal(round(LSpd,1)+" m/s") at (Col2Col,LSpdLine).
  }

local function DisplayRefresh
  {
// Refresh the MFD data on the display.
// Notes:
//    - This is to display the datas that keep changing.

    parameter Ap.
    parameter Pe.
    parameter Inc.
    parameter Ecc.
    parameter VSpd.
    parameter ASL.
    parameter AGL.
    parameter DTime.
    parameter TTime.
    parameter ManeuverStartTStmp.
    parameter CurrentTStmp.

    local DTimeFormatted to "Y"+DTime:year+"D"+DTime:day+" "+DTime:clock.

    print MFDVal(round(Ap/1000,3)+" km") at (Col1Col,ApLine). 
    print MFDVal(round(Pe/1000,3)+" km") at (Col1Col,PeLine).
    print MFDVal(round(Inc,1)+char(176)) at (Col1Col,IncLine).
    print MFDVal(round(Ecc,4)) at (Col1Col,EccLine).
    print MFDVal(round(VSpd,1)+" m/s") at (Col1Col,VSpdLine).
    print MFDVal(round(ASL/1000,3)+" km") at (Col1Col,ASLLine).
    print MFDVal(round(AGL/1000,3)+" km") at (Col1Col,AGLLine).
    print MFDVal(DTimeFormatted) at (Col2Col,DTimeLine).
    print MFDVal(TTime:full) at (Col2Col,TTimeLine).
    if ManeuverStartTStmp = timestamp(0)
      print MFDVal("") at (Col1Col,BnInLine).
    else
      {
        if ManeuverStartTStmp > CurrentTStmp
          print MFDVal("T-"+round(abs((CurrentTSTmp-ManeuverStartTStmp):seconds),1)+" s") at (Col1Col,BnInLine).
        else
          print MFDVal("T+"+round((CurrentTSTmp-ManeuverStartTStmp):seconds,1)+" s") at (Col1Col,BnInLine).
      }
  }

local function DisplayManeuver
  {
// Update the manuever data.

    parameter BnDu.
    parameter BnDv.

    print MFDVal(round(BnDu,2)+" s") at (Col1Col,BnDuLine).
    print MFDVal(round(BnDv,2)+" m/s") at (Col1Col,BnDvLine).
  }

local function DisplaySearchResults
  {
// Update the search results data.

    parameter FDTme.
    parameter FTTme.
    parameter FSMA.
    parameter FTDV.

    local FDTmeFormatted to "Y"+FDTme:year+"D"+FDTme:day+" "+FDTme:clock.

    print MFDVal(FDTmeFormatted) at (Col2Col,FDTmeLine).
    print MFDVal(FTTme:full) at (Col2Col,FTTmeLine).
    print MFDVal(round(FTDV,2)+" m/s") at (Col2Col,FTDVLine).

    if FSMA = VeryBigNumber
      print MFDVal("Infinity") at (Col2Col,FSMALine).
    else
      print MFDVal(round(FSMA/1000,3)+" km") at (Col2Col,FSMALine).

  }

local function DisplayFlightStatus
  {
// Update the Flight Status data.

    parameter stat.

    print stat:padright(ColSize) at (Col2Col,StatLine).
  }

local function DisplayError
  {
// Update the error info lines.
    parameter ErrorLine1.

    print ErrorLine1:tostring:padright(LineSize) at (0,Error1Line).
  }

local function DisplayDiagnostic
  {
// Update the diagnostic info lines.
    parameter DiagLine1.
    parameter DiagLine2.

    print DiagLine1:tostring:padright(LineSize) at (0,Diag1Line).
    print DiagLine2:tostring:padright(LineSize) at (0,Diag2Line).
  }

local function MFDVal
  {
// Format the Multi-function Display value.
// Pad the value from the left with spaces to right-align it.
// If the value is too large, truncate it from the left.
    parameter Val is "".

    local FmtVal is "".

    if Val:istype("Scalar")
      set Val to Val:tostring().

    if Val:length >= ColSize
      set FmtVal to Val:padright(ColSize).
    else
      set FmtVal to Val:padleft(ColSize).

    return FmtVal.
  }