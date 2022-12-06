// Name: Delta-vFunctions
// Author: JitteryJet
// Version: V02
// kOS Version: 1.3.2.0
// KSP Version: 1.11.2
// Description:
//    Functions to calculate values related to Delta-v.
//
// Notes:
//    -
//
// Todo:
//    -
//
// Update History:
//    24/07/2020 V01  - Created.
//    26/03/2021 V02  - Fixed up AN/DN switch issue. WIP.
//                    - Added Hohmann Transfer calculations.
//                    - Added "lazyglobal off"
//                    - Declare these functions GLOBAL to make it
//                      clear they are intended to be global in scope.
//    10/08/2021 V03  - Function name changes.
//                    -
@lazyglobal off.

global function DeltavEstBurnTime
  {
// Estimate of the burn time for a change in velocity of a
// rocket in an orbit.
// Notes:
//    - The equation allows for changes in mass as fuel is burnt.
//      Refer to the "Ideal Rocket Equation".
//    - The estimate assumes that thrust and ISP remain constant
//      during the burn.
// Todo:
//    -
    parameter Deltav.
    parameter MassInitial.  // In tonnes.
    parameter thrust.       // In kilonewtons.
    parameter ISP.

    local EffectiveExhaustVelocity to ISP*constant:g0.
    local MassFinal to MassInitial*constant:e^(-Deltav/EffectiveExhaustVelocity).
    local MassFuel to MassInitial-MassFinal.
    local FuelFlowRate to thrust/EffectiveExhaustVelocity.

    return MassFuel/FuelFlowRate.
  }

global function ISPVesselStage
  {
// Calculate the ISP of the current stage of the vessel.
// Notes:
//    - Lots of assumptions.
//    - Assumes all engines on a stage are the same? 
// Todo:
//    - 
    local ISP is 0.
    local englist is 0.
    list engines in englist.
    for eng in englist
      {
        if eng:stage = stage:number
          set ISP to ISP + eng:isp.
      }
    return ISP.
  }

global function CalcPlaneChangeDeltavVec
  {
// Calculate the direction and magnitude of the Delta-v required
// for a orbital plane change.
// Notes:
//    - Uses the "isosceles triangle" method of calculating the Delta-v.
//      This method changes the orbit inclination without
//      changing any other parts of the orbit (eg without changing
//      orbit eccentricity etc). 
// Todo:
//    - Test for retrograde orbits etc.

    parameter PositionVec.
    parameter VelocityVec.
    parameter InclinationChange.
    parameter NodeName.

    local DeltavVec to 0.

    local NormalVec to vcrs(PositionVec,VelocityVec):normalized.

    if NodeName = "AN"
      set DeltavVec to
        -NormalVec*VelocityVec:mag*sin(InclinationChange)
        -VelocityVec*(1-cos(InclinationChange)).
    else
    if NodeName = "DN"
      set DeltavVec to
        NormalVec*VelocityVec:mag*sin(InclinationChange)
        -VelocityVec*(1-cos(InclinationChange)).
    else
      print 0/0.

    return DeltavVec.
  }

global function CalcHohmannTransferDeltav
  {
// Calculate the delta-v required for a Hohmann Transfer.
// Notes:
//    - 
// Todo:
//    -

    parameter r1.  // Radius of the initital orbit.
    parameter r2.  // Radius of the final orbit.
    parameter mu.  // The Gravitational Parameter of the central body.

    local deltav to sqrt(mu/r1) * (sqrt(2 * r2/(r1 + r2)) - 1).

    return deltav.
  }

global function CalcHohmannCircularizationDeltav
  {
// Calculate the delta-v required for the circularization
// after a Hohmann Transfer.
// Notes:
//    - 
// Todo:
//    -

    parameter r1.  // Radius of the initital orbit.
    parameter r2.  // Radius of the final orbit.
    parameter mu.  // The Gravitational Parameter of the central body.

    local deltav to sqrt(mu/r2) * (1 - sqrt(2 * r1/(r1 + r2))).

    return deltav.
  }