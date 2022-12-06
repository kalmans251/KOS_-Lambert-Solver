// Name: LandSimpleLambertSolver
// Author: JitteryJet
// Version: V01
// kOS Version: 1.3.2.0
// KSP Version: 1.12.4
// Description:
//    Land the ship at a specified geoposition.
//
// Assumptions:
//    - No staging is required.
//    - The ship is in an elliptical orbit.
//    - The ship is in a prograde (anti-clockwise) orbit. The code MIGHT work with 
//      retrograde orbits but is untested.
//    - The body does not have an atmosphere. If the body does have an
//      atmosphere results will be unpredictable.
//
// Notes:
//
//    - This script finds a transfer orbit using a simple Lambert Solver.
//      "Short Way" elliptical transfer orbits only.
//      Single-revolution orbits only.
//
//    - This script does not attempt to find the BEST landing trajectory,
//      just an good solution in a reasonable amount of time.
//
//    - There is no guarantee the transfer orbit does not collide
//      with the surface before getting to the landing spot.
//      This is especially true of "LOWTIME" and "ASAP"
//      search types.
//
//    - The search filters out hyperbolic solutions.
//      On the KSP elliptical orbits can APPEAR to be
//      hyperbolic when the orbit goes outside the SOI of the body.
//
//    - Lots of orbits to keep track of:
//        Departure             - Ship's current orbit.
//        Transfer              - Ship's transfer orbit to the landing spot.      
//
//    - Abbreviations used in the orbital calculations (they are reasonably common):
//        a is the semi-major axis.
//        e is eccentricity.
//        E is eccentric anomaly.
//        F is hyperbolic eccentric anomaly.
//        M is mean anomaly.
//        mu is Standard Gravitational Parameter. 
//        nu is true anomaly.
//        r is radius of an orbit.
//        SOE is Specific Orbital Energy.
//        t is time.
//        v is speed (or velocity if a vector).
//
//    - The Lambert Solver algorithm is based mostly on the YouTube video series 
//      "AEE462 Lecture 10 - A Bisection Algorithm for the Solution of Lambert's Equation"
//      by M Peet, YouTube channel "Cybernetic Systems and Controls".  
//
// Todo:
//    - Re-evaluate timewarp undershoot/overshoot logic.
//      Found a case where the timewarp undershot by 8 minutes!
//    - Test on a body other than the Mun, especially one that
//      spins faster.
//    - Test a ship in a retrograde orbit.
//    - Test landing from a polar orbit (and any other extreme test cases).
//    - Optimise run time by reusing the Alpha and Beta angle values
//      in Lambert's Equation instead of recalculating them?
//      Benchmark change in performance to ensure it is worth doing.
//    - The search can find retrograde elliptical transfers.
//      They tend to be high deltav solutions that collide with the
//      surface.
//      Does it make sense to filter them out earlier to speed up the
//      search?
//    - Add staging (very difficult to get an accurate burn during maneuvers).
//    - Run the code through a Performance Analyser and optimize
//      if possible.
//    - Investigate compiling the code.
//    - Consider a safe mode that guarantees the selected transfer orbit
//      will not collide with the surface, the CREATEORBIT command might
//      be useful to filter out trajectories below a safe height.
//    -
//
// Update History:
//    13/11/2022 V01  - Created.
//                    -
//
@lazyglobal off.
// Increase IPU value to speed up scripts with a lot of calculations
// if the CPU and graphic card are good. Default is around 200.
// Max is around 2000.
set config:ipu to 2000.

// Parameter descriptions.        
//    LandingGeo              GeoPosition of the landing spot.
//    SearchType              Search type
//                              "ASAP"    - First valid transfer found.
//                              "LOWDV"   - Search for a low delta-v transfer.
//                              "LOWTIME" - Search for a low total trip time.
//    SearchSteps             Number of steps used by the search.
//                              The more steps the more chance the search will
//                              find a better solution but the longer the search
//                              will take.
//    MaxSearchSecs           Max amount of time the search is allowed to take (s).
//                              This determines at what point in the future the search
//                              will start. This is to avoid departure times in the past.
//    DescentHeightKm         Height above ground to start the descent (km).
//                              Choose a value high enough so the transfer orbit will clear the
//                              terrain between the departure point and the landing spot,
//                              and have enough magin of error to slow down for landing.
//    LandingHeight           Height above ground to ready vessel for landing (m).
//                              Choose a value to allow the landing gear to deploy.
//    LandingSpeed            Landing speed (m/s).
//    SteeringDuration        Time to allow the vessel to steer to the burn
//                            attitude for the maneuver (s).                            
//	  WarpType	  					  "PHYSICS","RAILS" or "NOWARP".
//    ShowArrows              "SHOW" or "NOSHOW".
//    

parameter LandingGeo to "".
parameter SearchType to "LOWDV".
parameter SearchSteps to 60.
parameter MaxSearchSecs to 120.
parameter DescentHeightKm to 10.
parameter LandingHeight to 100.
parameter LandingSpeed to 5.
parameter SteeringDuration to 60.
parameter WarpType to "NOWARP".
parameter ShowArrows to "NOSHOW".

// Load in library functions.
runoncepath("LandSimpleLambertSolverMFD V01").
runoncepath("Delta-vFunctions V03").
runoncepath("MiscFunctions V04").
runoncepath("OrbitFunctions V04").

local DescentHeight to DescentHeightKm*1000.
local NextMFDRefreshTime to time:seconds.
local ManeuverStartTStmp to timestamp(0).
local FatalError to false.
local MFDRefreshTriggerActive to true.
local VeryBigNumber to 3.402823E+38.
local SteeringTSpan to timespan(0,0,0,0,SteeringDuration).
local DiagnosticMN to 0.

local lock HeightAGL TO ship:altitude-ship:geoposition:terrainheight.

local tDepTStmp to time(0).
local tTransTSpan to timespan(0).
local aFinalTrans to 0.
local vFinalTransDv to 99999.
local tFinalTransTSpan to timespan(99999,0).
local tFinalDepTStmp to timestamp().

// Log file for debugging.
local LogFilename to kuniverse:realtime:tostring+".txt".

local DepartureArrow to
  vecdraw(V(0,0,0),V(0,0,0),red,"Departure",1,false,0.1,true,true).
local ArrivalArrow to
  vecdraw(V(0,0,0),V(0,0,0),green,"Arrival",1,false,0.1,true,true).
//local VacantFocusArrow to
//  vecdraw(V(0,0,0),V(0,0,0),yellow,"Vacant Focus",1,false,0.1,true,true).

sas off.
set ship:control:mainthrottle to 0.
clearvecdraws().
SetMFD().
CheckForErrorsAndWarnings().
if not FatalError
  {
    SearchForTransferOrbit().
    DoTransferOrbitManeuver().
    DoBrakingManeuver().
    DoLandingManeuver().
  }
MFDFunctions["DisplayFlightStatus"]("Finished").
RemoveLocksAndTriggers().

local function SearchForTransferOrbit
  {
// Search combinations of departure times and transfer times for
// a suitable transfer orbit that ends at the landing spot.
// Notes:
//    - Search for "Short Way" elliptical transfer orbits only.
//      This simplifies the code. But it will miss other solutions.
//    - The minimum transfer time defines the parabolic transfer orbit
//      solution: times greater than this give elliptical transfer orbits.
//    - The maximum transfer time defines the upper limit for the
//      "Short Way" solutions, transfer times longer than this
//      are "Long Way" solutions.
//    - The range of departure times and transfer times to search is
//      a guess at this point - it has to stop somewhere.
//      The search will find a solution in this range but it will
//      miss any better solutions from outside the range.
//    -
// Todo:
//    - Think some more about what is a good range of departure times
//      and transfer times to search.
//    - Consider adding "Long Way" solutions.
//    -

    MFDFunctions["DisplayFlightStatus"]("Transfer search").

    local tMaxSearchTSpan to timespan(0,0,0,0,MaxSearchSecs).
    local tDepFromTStmp to timestamp()+tMaxSearchTSpan.
    local tDepToTStmp to tDepFromTStmp+ship:obt:period.
    local tDepSearchStepTSpan to timespan(0,0,0,0,ship:obt:period/SearchSteps).

    local tTransToTSpan to timespan(0,0,0,0,ship:obt:period).
    local tTransSearchStepTSpan to timespan(0,0,0,0,ship:obt:period/SearchSteps).

    local r1 to 0.                          // Orbit radius of Point1.
    local r2 to 0.                          // Orbit radius of Point2.
    local c to 0.                           // Chord Point1-Point2.
    local mu to 0.                          // Standard Gravitational Parameter
    local aTrans to 0.                      // SMA of transfer orbit.
    local tMinTransTSpan to timespan(0).    // Minimum transfer time (TOF). Defines the parabolic solution.
    local tMaxTransTSpan to timespan(0).    // Maximum transfer time (TOF). Defines the short way solution.
    local aMinSOETrans to 0.                // Minimum energy transfer semi-major axis.
    local TransAng to 0.                    // Transfer angle.
    local vDeltaVec to 0.
    local TransferOrbitFound to false.

// Calculate vectors just prior to being used as
// KSP uses floating origins.
    local r1Vec to 0.
    local r2Vec to 0.

    set mu to ship:body:mu.
    if ShowArrows = "SHOW"
      {
        set DepartureArrow:show to true.
        set ArrivalArrow:show to true.
      }

    set tDepTStmp to tDepFromTStmp.
    until tDepTStmp > tDepToTStmp
      {
        set tTransTSpan to tTransSearchStepTSpan.
        until tTransTSpan > tTransToTSpan
          { 
            if tDepFromTStmp < timestamp()
              {
                MFDFunctions["DisplayError"]("Trying to use Departure times in the past").
                print 0/0.
              }
            set r1Vec to positionat(ship,tDepTStmp)-ship:body:position.
            set r2Vec to
              CalcGeoPositionAt(LandingGeo,DescentHeight,tDepTStmp+tTransTSpan)-ship:body:position.
            set TransAng to CalcAngleBetweenPositionVecs(r1Vec,r2Vec).
            set r1 to r1Vec:mag.
            set r2 to r2Vec:mag.
            set c to (r2Vec-r1Vec):mag.
            set tMinTransTSpan to CalcParabolicTransferTime(r1,r2,c,mu,TransAng).
            if ShowArrows = "SHOW"
              {
                set DepartureArrow:start to (2*r1Vec+ship:body:position).
                set DepartureArrow:vec to -r1Vec.
                set ArrivalArrow:start to (2*r2Vec+Ship:body:position).
                set ArrivalArrow:vec to -r2Vec.
              }
            if SearchType = "ASAP"
              {
                if tTransTSpan > tMinTransTSpan
                  {
                    set aMinSOETrans to (r1+r2+c)/4.
                    set tMaxTransTSpan to CalcTransferTimeLamberts(r1,r2,c,aMinSOETrans,mu,TransAng).
                    if tTransTSpan < tMaxTransTSpan
                      {
                        set TransferOrbitFound to true.
                        set tFinalDepTStmp to tDepTStmp.
                        set tFinalTransTSpan to tTransTSpan.
                        set aFinalTrans to CalcSMALamberts(tFinalTransTSpan,r1,r2,c,mu,TransAng).
                        set vDeltaVec to
                          CalcTransfervVec(r1Vec,r2Vec,aFinalTrans,mu)-velocityat(ship,tFinalDepTStmp):orbit.
                        set vFinalTransDv to vDeltaVec:mag.
                        MFDFunctions["DisplaySearchResults"]
                          (
                            tFinalDepTStmp,
                            tFinalTransTSpan,
                            aFinalTrans,
                            vFinalTransDv
                          ).
//                        DisplayDiagnosticMN(vDeltaVec,tDepTStmp).
// Finish the search.
                        set tTransTSpan to tTransToTSpan.
                        set tDepTStmp to tDepToTStmp.
                      }
                  }
              }
            else
            if SearchType = "LOWTIME"
              {
                if tTransTSpan > tMinTransTSpan
                  {
                    set aMinSOETrans to (r1+r2+c)/4.
                    set tMaxTransTSpan to CalcTransferTimeLamberts(r1,r2,c,aMinSOETrans,mu,TransAng).
                    if tTransTSpan < tMaxTransTSpan
                      {
                        set TransferOrbitFound to true.
                        if (tDepTStmp-tDepFromTStmp+tTransTSpan) < tFinalTransTSpan
                          {
                            set tFinalDepTStmp to tDepTStmp.
                            set tFinalTransTSpan to tTransTSpan.
                            set aFinalTrans to CalcSMALamberts(tFinalTransTSpan,r1,r2,c,mu,TransAng).
                            set vDeltaVec to
                              CalcTransfervVec(r1Vec,r2Vec,aFinalTrans,mu)-velocityat(ship,tFinalDepTStmp):orbit.
                            set vFinalTransDv to vDeltaVec:mag.
                            MFDFunctions["DisplaySearchResults"]
                              (
                                tFinalDepTStmp,
                                tFinalTransTSpan,
                                aFinalTrans,
                                vFinalTransDv
                              ).
                            MFDFunctions["DisplayDiagnostic"](tMinTransTSpan+" "+tMaxTransTSpan,round(TransAng,4)+" "+round(aMinSOETrans,4)).
                          }
//                        DisplayDiagnosticMN(vDeltaVec,tDepTStmp).
                      }
                  }
              }
            else
            if SearchType = "LOWDV"
              {
                if tTransTSpan > tMinTransTSpan
                  {
                    set aMinSOETrans to (r1+r2+c)/4.
                    set tMaxTransTSpan to CalcTransferTimeLamberts(r1,r2,c,aMinSOETrans,mu,TransAng).
                    if tTransTSpan < tMaxTransTSpan
                      {
                        set TransferOrbitFound to true.
                        set aTrans to CalcSMALamberts(tTransTSpan,r1,r2,c,mu,TransAng).
                        set vDeltaVec to
                          CalcTransfervVec(r1Vec,r2Vec,aTrans,mu)-velocityat(ship,tDepTStmp):orbit.
                        if vDeltaVec:mag < vFinalTransDv
                          {
                            set vFinalTransDv to vDeltaVec:mag.
                            set aFinalTrans to aTrans.
                            set tFinalDepTStmp to tDepTStmp.
                            set tFinalTransTSpan to tTransTSpan.
                            MFDFunctions["DisplaySearchResults"]
                              (
                                tFinalDepTStmp,
                                tFinalTransTSpan,
                                aFinalTrans,
                                vFinalTransDv
                              ).
                          }
                        DisplayDiagnosticMN(vDeltaVec,tDepTStmp).
                      }
                  }        
              }
            set tTransTSpan to tTransTSpan+tTransSearchStepTSpan.
            wait 0.
          }
        set tDepTStmp to tDepTStmp+tDepSearchStepTSpan.
      }
    set DepartureArrow:show to false.
    set ArrivalArrow:show to false.
    if not TransferOrbitFound
      {
        MFDFunctions["DisplayError"]("No transfer orbits found").
        print 0/0.
      }
  }

local function CalcSMALamberts
  {
// Calculate the semi-major axis given a transfer time using Lambert's Equation.
// Notes:
//    - Uses the Bisection algorithm.
//    - Elliptical orbits only.
//    - "Short Way" solutions only.
//    - This is only a simple implemetation of a Lambert Solver,
//      it will probably fail in complex situations?
//    - 
// Todo:
//    - It probably is not too much work to extend this algorithm
//      to handle "Long Way" solutions as well.
//    -     

    parameter t.        // Transfer time (TOF) from Point1 to Point2.
    parameter r1.       // Orbit radius at Point1.
    parameter r2.       // Orbit radius at Point2.
    parameter c.        // Chord Point1-Point2.
    parameter mu.       // Standard Gravitational Parameter of central body.
    parameter TransAng. // Transfer angle.

// Bisection Algorithm stopping criteria.
    local TolerancePct to 0.1.
    local tToleranceTSpan to t*TolerancePct/100.

    local a to 0.       // Semi-major axis.
    local s to          // Semi-perimeter.
      (r1+r2+c)/2.
    local amin to 0.    // Bisection minimum a.
    local amax to 0.    // Bisection maximum a.

    local finished to false.
    local tcalc to 0.

// Initial guess.
    set amin to s/2.
    set amax to 2*s.

// Adjust initial amax guess if it isn't high enough.
    set tcalc to CalcTransferTimeLamberts(r1,r2,c,amax,mu,TransAng).
    until tcalc < t
      {
        set amax to amax*2.
        set tcalc to CalcTransferTimeLamberts(r1,r2,c,amax,mu,TransAng).
      }

// Find the semi-major axis for the given transfer time.
    set finished to false.
    until finished
      {
        set a to (amax+amin)/2.
        set tcalc to CalcTransferTimeLamberts(r1,r2,c,a,mu,TransAng).
        if tcalc > t
          set amin to a.
        else
          set amax to a.
        set finished to NearEqual(t:seconds,tcalc:seconds,ttoleranceTSpan:seconds).
      }
    return a.
  }

local function CalcTransferTimeLamberts
  {
// Calculate transfer time using Lambert's Equation.
// Notes:
//    - Uses the Lagrange Formulation of Lambert's Equation
//      copied verbatim from various sources.
//    - Only works for elliptical orbit transfers ie not
//      parabolic or hyperbolic.
//    - The code is written as a series of steps to make it
//      easier to understand and debug.
//    - 
// Todo:
//    -

    parameter r1.             // Orbit radius at Point1.
    parameter r2.             // Orbit radius at Point2.
    parameter c.              // Chord Point1-Point2.
    parameter a.              // Semi-major axis.
    parameter mu.             // Standard Gravitational Parameter of central body.
    parameter TranferAng.     // Transfer angle.

// Semi-perimeter.
    local s to (r1+r2+c)/2.

// The "alpha" and "beta" terms in the equation.
// I don't know why they break it down like this, except maybe
// to put it into a "form" similiar to Kepler's Equation.
    local AlphaDeg to 2*arcsin(sqrt(s/(2*a))).
    local AlphaRad to AlphaDeg*constant:DegToRad.
    local BetaDeg to 2*arcsin(sqrt((s-c)/(2*a))).
    local BetaRad to BetaDeg*constant:DegToRad.

    if TranferAng > 180
      {
        set BetaDeg to -BetaDeg.
        set BetaRad to -BetaRad.
      }

// Transfer time.
    local t to
      sqrt(a^3/mu)*(AlphaRad-BetaRad-(sin(AlphaDeg)-sin(BetaDeg))).

    return timespan(0,0,0,0,t).
  }

local function CalcParabolicTransferTime
  {
// Calculate the parabolic transfer time.
// Notes:
//    - The parabolic solution defines the minimum transfer
//      time. For an elliptical orbit solution,
//      the transfer time has to be greater than this.
//    - Another way of looking at it is a parabolic
//      solution has an SMA value that approaches infinity.
//      Solutions close to the parabolic solution will
//      also have large SMA values.
//    -
// Todo:
//    -

    parameter r1.       // Orbit radius at Point1.
    parameter r2.       // Orbit radius at Point2.
    parameter c.        // Chord Point1-Point2.
    parameter mu.       // Standard Gravitational Parameter of central body.
    parameter TransAng. // Transfer angle.

// Semi-perimeter.
    local s to (r1+r2+c)/2.

    local sign to 1.

    if TransAng > 180
      set sign to -sign.

    local tparabolic to
      (sqrt(2)/3)*sqrt(s^3/mu)*(1-sign*((s-c)/s)^1.5).

    return timespan(0,0,0,0,tparabolic).
  }

local function CalcTransfervVec
  {
// Calculate the velocity of the transfer orbit at
// the departure point.
// Notes:
//    - This works because the departure(r1) and arrival(r2)
//      position vectors define the orbital plane for the transfer orbit.
//    - Once the transfer velocity at the departure point is known,
//      the delta-v for the maneuver can be calculated.
//    - The velocity at the arrival point can also be calculated using
//      a similiar equation (not implemented in this script).
//    - 
// Todo:
//    - 
//    -

    parameter r1Vec.
    parameter r2Vec.
    parameter a.
    parameter mu.

// Cord vector.
    local cVec to r2Vec-r1Vec.

// Cord.
    local c to cVec:mag.

// Semi-perimeter.
    local s to (r1Vec:mag+r2Vec:mag+c)/2.

// Transfer angle.
    local TransferAng to CalcAngleBetweenPositionVecs(r1Vec,r2Vec).

// The same alpha and beta parameters used in
// Lambert's Equation.
    local AlphaDeg to 2*arcsin(sqrt(s/(2*a))).
    local BetaDeg to 2*arcsin(sqrt((s-c)/(2*a))).

    if TransferAng > 180
      set BetaDeg to -BetaDeg.

// A and B are also parameters (I guess).
    local ACap to sqrt(mu/(4*a))*CalcCot(AlphaDeg/2).
    local BCap to sqrt(mu/(4*a))*CalcCot(BetaDeg/2).

    local TransfervVec to
      (BCap+ACap)*cVec:normalized+(BCap-ACap)*r1Vec:normalized.

    return TransfervVec.
  }

local function DoTransferOrbitManeuver
  {
// Do the maneuver that puts the ship into a transfer orbit.
// Notes:
//    - 
// Todo:
//    - 

    local ThrottleSet to 0.
    local SteeringDir to 0.
    local ManeuverSecs to 0.
    local SteeringStartTStmp to 0.
    local ManeuverVec to 0.

    set ManeuverSecs to
      DeltavEstBurnTime
        (
          vFinalTransDv,
          ship:mass,
          ship:availablethrust,
          ISPVesselStage()
        ).

    set ManeuverStartTStmp to tFinalDepTStmp-ManeuverSecs/2.
    set SteeringStartTStmp to ManeuverStartTStmp-SteeringTSpan.
    MFDFunctions["DisplayFlightStatus"]("Departure wait").
    MFDFunctions["DisplayManeuver"](ManeuverSecs,vFinalTransDv).
    if timestamp() > SteeringStartTStmp
      {
        MFDFunctions["DisplayError"]("Not enough time to do maneuver").
        print 0/0.
      }
    else
      DoSafeWait(SteeringStartTStmp,WarpType).

// Recalculate the delta-v vector again just prior to being used as
// KSP uses floating origins.
    set ManeuverVec to
      CalcTransfervVec
        (
          positionat(ship,tFinalDepTStmp)-ship:body:position,
          CalcGeoPositionAt(LandingGeo,DescentHeight,tFinalDepTStmp+tFinalTransTSpan)-ship:body:position,
          aFinalTrans,
          ship:body:mu
        )
      -velocityat(ship,tFinalDepTStmp):orbit.

    set SteeringDir to lookDirUp(ManeuverVec,ship:facing:topvector).
    lock steering to SteeringDir.
    lock throttle to Throttleset.

    MFDFunctions["DisplayFlightStatus"]("Steering").
    wait until timestamp() > ManeuverStartTStmp.
    MFDFunctions["DisplayFlightStatus"]("Transfer burn").
    set Throttleset to 1.
    wait until timestamp() > ManeuverStartTStmp+ManeuverSecs.
    set ThrottleSet to 0.
    MFDFunctions["DisplayFlightStatus"]("Transfering").
    set ManeuverStartTStmp to timestamp(0).
    MFDFunctions["DisplayManeuver"](0,0).

// Wait to allow throttle down to complete.
// If this is not done, subsequent "on rails" time warps might
// fail due to phantom acceleration.
    wait 0.5.

    unlock throttle.
    unlock steering.
  }

local function DoBrakingManeuver
  {
// Do the braking maneuver to slow down the ship at the
// end of the transfer in preparation for landing.
// Notes:
//    - This function assumes the ship was following
//      the transfer trajectory before the braking burns start.
//    - The braking maneuver includes a course correction
//      towards the landing spot.
//    - Using a Lambert Solver like this is EXPERIMENTAL.
//    -
//
// Todo:
//    - Test using a low thrust ship.
//    - Test on a body where the orbital velocity of the surface
//      is high (The Mun and Minmus are slow).
//    - Think some more about using a Lambert Solver like this,
//      it may not work in some situations?
//    - Think some more about when to start the throttle down.
//      Too soon wastes fuel hovering. Too late will cause the
//      landing spot to be overshot.
//      An overshoot and the resulting steering flipping can
//      happen easily. The algorithm needs to handle overshoots more
//      reliably. Perhaps a PID loop?
//    -

    local ManeuverVec to 0.
    local ManeuverSecs to 0.
    local ShipRoofVec to 0.
    local SteeringDir to 0.
    local r1 to 0.
    local r2 to 0.
    local c to 0.
    local TransAng to 0.
    local TransEndTStmp to 0.
    local ThrottleSet to 0.
    local aMinSOETrans to 0.
    local tMaxTransTSpan to 0.
    local tMinTransTSpan to 0.
    local finished to false.
    local vTransVec to 0.
    local r1Vec to 0.
    local r2Vec to 0.
    local tStopDeadSecs to 0.
    local TurnoverVec to 0.
    local aTrans to 0.
    local RemainingBurnDurationThrottledownSecs to 1.

// Redefine the "height above the ground" from the COM of the ship
// to the bottom of the vessel. This was delayed until now
// so the ship configuration and bounding box are in their final
// state (ignoring the landing legs deployment).
    local BBox to ship:bounds.
    lock HeightAGL to BBox:bottomaltradar.

    lock throttle to ThrottleSet.

    set TransEndTStmp to tFinalDepTStmp+tFinalTransTSpan.

// Start the ship turnover for the braking maneuvers at a suitable
// distance before the end of the transfer.
// In this case the distance travelled coming to a dead stop
// is used. The actual time required to do the steering and
// braking will usually(?) be less than this amount.
// The traditional rule of thumb is the lead time required to
// deaccelerate to a dead stop is around half the burn time
// (it's the triangle rule or something, cannot remember).
// 
    set TurnoverVec to velocityat(ship,TransEndTStmp):orbit.
    set tStopDeadSecs to
      DeltavEstBurnTime
        (
          TurnoverVec:mag,
          ship:mass,
          ship:availablethrust,
          ISPVesselStage()
        ).
    set ManeuverStartTStmp to TransEndTStmp-tStopDeadSecs/2.
    if timestamp() > ManeuverStartTStmp-SteeringDuration
      {
        MFDFunctions["DisplayError"]("Not enough time for steering and braking").
        print 0/0.
      }
    else
      DoSafeWait(ManeuverStartTStmp-SteeringDuration,WarpType).
    MFDFunctions["DisplayDiagnostic"](tStopDeadSecs,"").
//    kuniverse:pause().

    MFDFunctions["DisplayFlightstatus"]("Steering").
    set ShipRoofVec to ship:facing:topvector.
    set SteeringDir to lookdirup(-TurnoverVec,ShipRoofVec).
    lock steering to SteeringDir. 
    wait until timestamp() > ManeuverStartTStmp.
    set ManeuverStartTStmp to timestamp(0).

    MFDFunctions["DisplayFlightstatus"]("Braking burns").

// Use a Lambert Solver to continually calculate an elliptical
// orbit trajectory from the ship to the predicted position
// of the landing spot (keep in mind the landing spot is moving
// in the orbital frame). The predicted position will converge
// to the actual position when the ship get there.
//
// The purpose of the braking burn is to slow the ship down
// and to apply a course correction towards the landing spot.
//
// The "beauty" of using a Lambert Solver like this is the
// ship will follow a "short way" elliptical trajectory which
// will smoothly rotate the ship from horizontal to vertical;
// and because a strong vector is defined at all times the ship
// will not suddenly become unstable or flip as the speed nears zero.
    set tDepTStmp to TransEndTStmp-tStopDeadSecs.
    set tTransTSPan to TransEndTStmp-tDepTStmp.
    until finished
      {
        wait 0. // Ensure the physics click stuff is up to date.
        set r1Vec to positionat(ship,tDepTStmp)-ship:body:position.
        set r2Vec to CalcGeoPositionAt(LandingGeo,0,tDepTStmp+tTransTSpan)-ship:body:position.
        set r1 to r1Vec:mag.
        set r2 to r2Vec:mag.
        set c to (r2Vec-r1Vec):mag.
        set TransAng to CalcAngleBetweenPositionVecs(r1Vec,r2Vec).
        set aMinSOETrans to (r1+r2+c)/4.
        set tMaxTransTSpan to CalcTransferTimeLamberts(r1,r2,c,aMinSOETrans,ship:body:mu,TransAng).
        set tMinTransTSpan to CalcParabolicTransferTime(r1,r2,c,ship:body:mu,TransAng).
        MFDFunctions["DisplayDiagnostic"](tMinTransTSpan+" "+tMaxTransTSpan,round(TransAng,4)+" "+round(aMinSOETrans,4)).

// A failsafe test in case the transfer time "guess" is no longer valid.
// I saw this happen during testing when the ship overshot the landing spot.
        if tTransTSpan < tMinTransTSpan
          or tTransTSpan > tMaxTransTSpan
          {
            MFDFunctions["DisplayError"]("No valid transfer orbit for transfer time").
            print 0/0.
           }
        set aTrans to CalcSMALamberts(tTransTSpan,r1,r2,c,ship:body:mu,TransAng).
        set vTransVec to
          CalcTransfervVec
            (
              r1Vec,
              r2Vec,
              aTrans,
              ship:body:mu
            ).
        set ManeuverVec to vTransVec-velocityat(ship,tDepTStmp):orbit.
        set ManeuverSecs to
          DeltavEstBurnTime
            (
              ManeuverVec:mag,
              ship:mass,
              ship:availablethrust,
              ISPVesselStage()
            ).
        MFDFunctions["DisplaySearchResults"]
          (
            tDepTStmp,
            tTransTSpan,
            aTrans,
            vTransVec:mag
          ).
        MFDFunctions["DisplayManeuver"](ManeuverSecs,ManeuverVec:mag).
        set SteeringDir to lookdirup(ManeuverVec,ShipRoofVec).

// A poor-man's proportional thrust controller.
        set ThrottleSet to
          min(ManeuverSecs/RemainingBurnDurationThrottledownSecs,1).
        set tDepTStmp to timestamp().

// Guessing a transfer time like this was an act of desperation
// on my part, but it worked. There might be a more sensible
// way of guessing it.
        set tTransTSPan to (tMinTransTSpan+tMaxTransTSpan)/2.
        if HeightAGL < LandingHeight
          set finished to true.
      }
  }

local function DoLandingManeuver
  {
// Do the landing maneuver.
// Notes:
//    - This function assumes the ship is descending
//      vertically at this point - if not then results
//      will be unpredictable.
//    -
//
// Todo:
//    - 

    local SpeedPID to PIDLoop().
    set SpeedPID:kp to 0.1.
    set SpeedPID:ki to 0.1.
    set SpeedPID:minoutput to 0.
    set SpeedPID:maxoutput to 1.

    MFDFunctions["DisplayFlightstatus"]("Landing").
    legs on.
    set SpeedPID:setpoint to -LandingSpeed.
    lock throttle to SpeedPID:update(time:seconds,ship:verticalspeed).
    wait until ship:status="LANDED".
    MFDFunctions["DisplayFlightstatus"]("Landed").
    unlock throttle.
// Stabilize the vessel.
    wait 5.
    unlock steering.
  }

local function CalcGeoPositionAt
  {
// Calculate the position vector of the spot above (or below)
// a geographical position at some time in the future.
// Notes:
//    -
// Todo:
//    -

    parameter Geo.
    parameter AboveHeight.
    parameter Tstmp.

    local FutureLng to
      Geo:lng+((Tstmp-timestamp()):seconds/ship:body:rotationperiod)*360.

    local FuturePositionVec to
      latlng(Geo:lat,FutureLng):altitudeposition(Geo:terrainheight+AboveHeight).

    return FuturePositionVec.
  }

local function SetMFD
  {
// Set the Multi-function Display.
// Notes:
//    -
// Todo:
//    -
    clearScreen.
    set terminal:width to 56+1.
//    set terminal:width to 100.
    set terminal:height to 27.
    MFDFunctions["DisplayLabels"]
      (
        ship:name,
        SearchType,
        SearchSteps,
        LandingGeo:lat,
        LandingGeo:lng,
        DescentHeight,
        LandingHeight,
        LandingSpeed
      ).
    SetMFDRefreshTrigger().
  }

local function SetMFDRefreshTrigger
  {
// Refresh the Multi-function Display.
// Notes:
//		-
// Todo:
//		- Try to figure out how often this needs to run.
//    - It should be easy enough to add logic to skip a number of physics
//      ticks before a refresh is done if necessary.
//    -
    local RefreshInterval to 0.1.
    when (NextMFDRefreshTime < time:seconds)
    then
      {
        MFDFunctions["DisplayRefresh"]
         (
          ship:obt:apoapsis,
          ship:obt:periapsis,
          ship:obt:inclination,
          ship:obt:eccentricity,
          ship:verticalspeed,
          ship:altitude,
          HeightAGL,
          tDepTStmp,
          tTransTSpan,
          ManeuverStartTStmp,
          timestamp()
         ).
        set NextMFDRefreshTime to NextMFDRefreshTime+RefreshInterval.
        return MFDRefreshTriggerActive.
      }
  }

local function CheckForErrorsAndWarnings
  {
// Check for errors and warnings.
// Notes:
//    - I picked "reasonable" values to check for.
// Todo:
//    - Add warning about atmospheres.

    if SearchType <> "ASAP"
      and SearchType <> "LOWDV"
      and SearchType <> "LOWTIME"
      {
        MFDFunctions["DisplayError"]("Search Type is invalid").
        set FatalError to true.
      }
  }

local function RemoveLocksAndTriggers
  {
// Remove locks and triggers.
// Notes:
//    - Guarantee unneeded locks and triggers are removed before
//      any following script is run. THROTTLE, STEERING and
//      triggers are global and will keep processing
//      until control is returned back to the terminal program -
//      this is relevant if this script is ran using
//      RUNPATH from another script before exiting to the
//      terminal program.
//    -
// Todo:
//    -

// Set the triggers to fire once more only.
    set MFDRefreshTriggerActive to false.

// Ensure the triggers finish firing once more then stop.
    wait 0.

// Remove any global variables that might
// cause problems if they hang around.
    unset MFDFunctions.

// Unlock the throttle and steering controls
// used by the Player.
    unlock throttle.
    unlock steering.

// One more physics tick before finishing this script,
// just to be on the safe side.
    wait 0.
  }

local function CalcCot
  {
// Cotangent function input degrees.

    parameter angle.

//    local cotx to cos(angle)/sin(angle).
    local cotx to 1/tan(angle).

    return cotx. 
  }

local function CreateNodeFromVec
  {
// Create a maneuver node from a delta-v vector.
// Notes:
//    - This code was copied from the Internet.
//    -
// Todo:
//    - This function needs to be checked and added to the code library.
//    -
    PARAMETER vec.
    parameter n_time IS TIME:SECONDS.

    LOCAL s_pro IS VELOCITYAT(SHIP,n_time):ORBIT.
    LOCAL s_pos IS POSITIONAT(SHIP,n_time)-ship:BODY:POSITION.
    LOCAL s_nrm IS VCRS(s_pro,s_pos).
    LOCAL s_rad IS VCRS(s_nrm,s_pro).

    LOCAL pro IS VDOT(vec,s_pro:NORMALIZED).
    LOCAL nrm IS VDOT(vec,s_nrm:NORMALIZED).
    LOCAL rad IS VDOT(vec,s_rad:NORMALIZED).

    RETURN NODE(n_time, rad, nrm, pro).
  }

local function DisplayDiagnosticMN
  {
// Display a KSP maneuver node as a diagnostic.
// Notes:
//    - The actual orbit will closely match this node
//      after the maneuver burn is done.
//    - Keep in mind running this diagnostic will
//      slow down the search, increasing the time
//      allowed for the search may be necessary.
//    - If you fiddle around with this code only display
//      a node for a short period of time then remove it
//      as a node on the flightpath of the ship affects the
//      orbit prediction commands such as positionat and
//      velocityat.
//    -
// Todo:
//    -
//
    parameter MNVec.
    parameter MNTStmp.

    set DiagnosticMN to CreateNodeFromVec(MNVec,MNTStmp).
    add DiagnosticMN.
    wait 1.
//    kuniverse:pause().
    remove DiagnosticMN.
    wait 0.
  }

local function DoSafeWait
  {
// Wait until a point in time.
// Notes:
//    - Work in progress until I am happy it is indeed "safe" enough.
//      Once safe the function will be added to the function library.
//    - Waiting until a point in time is usually safer than waiting a
//      number of game seconds.
//    - The KSP Time Warp is a "4th Wall" function as far as
//      kOS is concerned. Time Warp runs independently of kOS and can
//      respond to user input. Time Warp is not well synchronised
//      with kOS.
//    - This function tries to allow for various factors that
//      can affect how well kOS and Time Warp run together.
//    -
// Todo
//    - Test, test and test some more.
//    - Allow a timing margin to avoid Time Warp undershoot and
//      overshoot?
//    -

    parameter WaitToTStmp.
    parameter WarpType.

    if WarpType = "NOWARP"
      wait until timestamp() > WaitToTStmp.
    else
    if WarpType = "PHYSICS"
      {
        set kuniverse:timewarp:mode to WarpType.
        kuniverse:timewarp:warpTo(WaitToTStmp:seconds).
        wait until timestamp() > WaitToTStmp.
      }
    else
    if WarpType = "RAILS"
      {
// On-rails time warping only runs a limited game simulation,
// the ship is unpacked, some system values become undefined etc.
// The Player can also change the warp rate or stop the time warp completely.
// The "wait until" tries to get the time warp and the kOS script back into
// sync without issues, at the expense of a possible overshoot.
// Check the KSP log to see time warp undershoot/overshoot warnings.
        set kuniverse:timewarp:mode to WarpType.
        kuniverse:timewarp:warpTo(WaitToTStmp:seconds).
        wait until kuniverse:timewarp:warp = 0 and ship:unpacked.

// This "wait until" runs independently of the time warp.
// This wait will still work even if the time warp is modified by
// Player input.
// If the time warp stops early (undershoots) the wait duration will
// still be correct. If the time warp stops late (overshoots) then
// the wait duration will be longer than expected.
        wait until timestamp() > WaitToTStmp.
      }
    else
      print 0/0.  // Unknown WarpType value so terminate the script.
  }