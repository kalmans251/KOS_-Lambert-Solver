// Name: MiscFunctions
// Author: JitteryJet
// Version: V04
// kOS Version: 1.3.2.0
// KSP Version: 1.11.2
// Description:
//    Miscellaneous Functions.
//
// Notes:
//    - Only generic fully parameterised functions.
//    -
// Todo:
//    -
//    -
// Update History:
//    24/07/2020 V01  - Created.
//    26/03/2021 V02  - Added "lazyglobal off"
//                    - Declare these functions GLOBAL to make it
//                      clear they are intended to be global in scope.
//    30/04/2021 V03  - Added function to calculate the intersection point
//                      between a line and a plane.
//    26/04/2021 V04  - Added clamp function.
//                    -
//
@lazyglobal off.

global function NearEqual
  {
// True if two values are equal within a specified margin.
    parameter value1.
    parameter value2.
    parameter margin.

    if value1 >= value2 - margin and value1 <= value2 + margin
      return true.
    else
      return false.
  }

global function WarpToTime
  {
// Warp to a point in time.
// Notes:
//    -
// Todo:
    parameter ToTime.
    parameter mode.

    set kuniverse:timewarp:mode to mode.
    kuniverse:timewarp:warpto(ToTime).
  }

global function CalcLinePlaneIntersection
  {
// Calculate the intersection position between a line and a plane.
// Notes:
//    - Returns the intersection position if there is one.
//    - Returns V(0,0,0) if the line lies on the
//      plane or the line does not intersect the plane. 
//    - The equation is from Wikipedia. It appears to be the
//      common Algebraic Form for vectors.
//    - Example usage is to predict the impact point ahead of a
//      vessel heading towards the surface of a body. The "plane" is defined
//      as the surface directly under vessel, the line is the velocity vector.
//      This ignores the curvature of the body.
//    -
// Todo:
//    -
    parameter LineVec.          // Vector defining the line. 
    parameter PlaneNormalVec.   // Vector defining the normal line to the plane.
    parameter LinePos.          // Position defining the point on the line.
    parameter PlanePos.         // Position defining the point on the plane.

    local denominator to vdot(LineVec,PlaneNormalVec).

    if denominator = 0
      return V(0,0,0).
    else
      return
        (vdot(PlanePos-LinePos,PlaneNormalVec)/denominator)
        *LineVec+LinePos.
  }

global function clamp
  {
// Graphics clamp.
// Notes:
//    -
// Todo:
//    -
    parameter x.
    parameter min.
    parameter max.

    if x < min
      set x to min.
    else
    if x > max
      set x to max.
    return x.
  }