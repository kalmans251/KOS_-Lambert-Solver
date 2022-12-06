// Name: OrbitFunctions
// Author: JitteryJet
// Version: V04
// kOS Version: 1.3.2.0
// KSP Version: 1.12.4
// Description:
//    Functions to calculate values related to orbits.
//
// Notes:
//    - SAM is the Specific Angular Momentum pseudovector for an orbit.
//    -
//
// Todo:
//    -
//
// Update History:
//    24/07/2020 V01  - Created.
//    01/04/2021 V02  - Fixed up AN DN swap error.
//                    - Added Hohmann Transfer calculations.
//                    - Fixed up error in AN and DN position calc
//                      in some quadrants.
//    10/08/2021 V03  - Removed the term 'phase' as it is meaningless
//                      in the context of Hohmann transfer orbits.
//                    - Normalized the Hohmann transfer angle.
//    13/11/2022 V04  - Added CalcOrbitRadius.
//                    - Added CalcAngleBetweenPositionVecs.
//                    - 

global function TrueToEccentricAnomaly
  {
// Convert a true anomaly to an eccentric anomaly.
// Notes:
//    - 
// Todo:
//    - Test for true anomaly values >= 360.
//    -

    parameter TrueAnomaly.
    parameter eccentricity.

    local EccentricAnomaly is arccos((eccentricity+cos(TrueAnomaly))/(1+eccentricity*cos(TrueAnomaly))).

    if TrueAnomaly > 180
      set EccentricAnomaly to 360-EccentricAnomaly.

    return EccentricAnomaly.
  }

global function TrueToMeanAnomaly
  {
// Convert a true anomaly into a mean anomaly.
// Notes:
//    - It's magic. It works by using the "eccentric anomaly" as an intermediate variable.
//    - kOS orbital angles are given in degrees. These must be converted to radians
//      for this formula to work.
// Todo:
//    - 

    parameter TrueAnomaly.
    parameter eccentricity.

    local EccentricAnomaly to TrueToEccentricAnomaly(TrueAnomaly,eccentricity).
    local EccentricAnomalyRad to EccentricAnomaly*constant:DegToRad.

    local MeanAnomalyRad to EccentricAnomalyRad-(eccentricity*sin(EccentricAnomaly)).

    return MeanAnomalyRad*constant:RadtoDeg.
  }

global function TimeToOrbitPosition
  {
// Estimate the time for an object to orbit from an initial position to a final position.
// Notes:
//    - 
// Todo:
//    - 

    parameter InitialTrueAnomaly.
    parameter FinalTrueAnomaly.
    parameter eccentricity.
    parameter period.

    local t to 0.
    local InitialMeanAnomaly to TrueToMeanAnomaly(InitialTrueAnomaly,eccentricity).
    local FinalMeanAnomaly to TrueToMeanAnomaly(FinalTrueAnomaly,eccentricity).

    if InitialMeanAnomaly <= FinalMeanAnomaly
      set t to (FinalMeanAnomaly-InitialMeanAnomaly)/360 * period.
    else
      set t to (FinalMeanAnomaly-InitialMeanAnomaly+360)/360 * period.

    return t.
  }

global function CalcAscendingNodeVec
  {
// Calculate the ascending node vector for one orbit
// relative to another orbit.
// Notes:
//    - The orbitals share a central body ie they
//      are both in the same KSP SOI.
//    - The vector defines a line from the centre of the body
//      that passes through the ascending node on the orbit.
// Todo:
//    - Try and understand WHERE the line of nodes vector should
//      point according to the Left Hand Rule or whatever.
//      The AN and DN are swapped??
      
    parameter Orbit1SAMVec.
    parameter Orbit2SAMVec.
    
    return vcrs(Orbit1SAMVec,Orbit2SAMVec).
  }

global function CalcRelativeInclination
  {
// Calculate the inclination of this orbit
// to another orbit.
// Notes:
//    - The orbits share a central body ie they
//      are both in the same KSP SOI.

    parameter Orbit1SAMVec.
    parameter Orbit2SAMVec.
    
    return vang(Orbit1SAMVec,Orbit2SAMVec).
  }

global function CalcEccentricityVec
  {
// Calculate the Eccentricty Vector for an orbit.
// Notes:
//    - Think of this as the line from the
//      centre of the body to the periapsis.
//    - The magnitude of the vector is the eccentricity
//      of the orbit.
//    - It also defines the Line of Apsides.
// Todo
//    - 
    parameter PositionVec.
    parameter VelocityVec.
    parameter mu.

    return
      (VelocityVec:mag^2/mu-1/PositionVec:mag)*PositionVec-
      vdot(PositionVec,VelocityVec)/mu*VelocityVec.
//    return
//      vcrs(VelocityVec,SAMVec)/mu - VelocityVec/VelocityVec:mag.

  }

global function CalcSAMVec
  {
// Calculate the Specific Angular Moment pseudovector
// for an orbit.
// Notes:
//    - Think of this vector as the "axis" of the orbit,
//      with the origin being the centre of the body.
//    - This vector is normal to the plane of the orbit.
//
// Todo:
//    - 
    parameter PositionVec.
    parameter VelocityVec.

    return vcrs(PositionVec,VelocityVec).
  }

global function CalcTrueAnomalyFromVec
  {
// Calculate the true anomaly angle from a true anomaly vector pointing
// from the centre of the body to a point in the orbit.
// Notes:
//    - Returns an angle 0-360 degress.
//    - Assumes the vector lies on the orbital plane.
//    - I found this code on the Internet.
//
// Todo:
//    - Try and find a less complicated method that does
//      not require the normal to the orbit parameter.

    parameter TrueAnomalyVec.
    parameter EccentricityVec.
    parameter OrbitNormalVec.

// Calculate the smaller angle between the point on the orbit and the periapsis.
    local angle to vang(TrueAnomalyVec,EccentricityVec).

// Determine which quadrant the point on the orbit lies in,
// and adjust the angle to the correct value.
    if vang(TrueAnomalyVec,vcrs(OrbitNormalVec,EccentricityVec)) < 90
      return angle.
    else
      return 360-angle. 
  }

global function CalcHohmannOrbitAngle
  {
// Calculate the required orbit angle between the initial
// orbital and target orbital for a Hohmann Transfer.
// Notes:
//    - The angle is normalized to 0-360 degrees.
// Todo
//    -

    parameter r1.  // Radius of the initital orbit.
    parameter r2.  // Radius of the final orbit.

// Formula from Wikipedia.
//    local angle to constant:pi*(1-(1/(2*sqrt(2)))*sqrt((r1/r2+1)^3)).

// Formula from a cheat sheet I found.
    local angle to constant:pi*(1-((r1+r2)/(2*r2))^1.5).
    set angle to mod(angle,2*constant:pi).
    if angle < 0
      set angle to angle+2*constant:pi.
    return angle*constant:RadToDeg.
  }

global function CalcOrbitRadius
  {
// Calculate the radius of an orbit at
// a specified true anomaly.
// Notes:
//    - WIP.
//    - 
// Todo:
//    -
    parameter TrueAnomaly.
    parameter Eccentricity.
    parameter SemiMajorAxis.

    return
      SemiMajorAxis*(1-eccentricity^2)/(1+eccentricity*cos(TrueAnomaly)).

  }

global function CalcAngleBetweenPositionVecs
  {
// Calculate the angle (0 < angle < 360) between two
// position vectors.
// Notes:
//    - Code found on the Internet. Use with caution
//      as it is not sufficiently tested.
//    - Technically-speaking the angle between two vectors
//      is defined as 0 < angle < 180. It can be expanded to
//      360 degrees by using the convention that the angle
//      increases in the prograde (anti-clockwise) direction.
//    - Keep in mind KSP uses a left-handed coordinate system.
//    -
// Todo:
//    - 

    parameter r1Vec.
    parameter r2Vec.

    local angle to vang(r1Vec,r2Vec).

    if vcrs(r2Vec,r1Vec):y < 0
      set angle to 360-angle.

    return angle.
  }