#include "mujoco.h"
/**
 * Determines the point of intersection between a plane defined by a point and a normal vector and a line defined by a point and a direction vector.
 *
 * @param planePoint    A point on the plane.
 * @param planeNormal   The normal vector of the plane.
 * @param linePoint     A point on the line.
 * @param lineDirection The direction vector of the line.
 * @return The point of intersection between the line and the plane, null if the line is parallel to the plane.
 */
bool lineIntersection(mjtNum planePoint[3], mjtNum planeNormal[3], mjtNum linePoint[3], mjtNum lineDirection[3], mjtNum interPoint[3]) 
{
    mju_normalize3(lineDirection);
    if (mju_dot3(planeNormal,lineDirection) == 0) {
        return false;
    }
    double t = (mju_dot3(planeNormal,planePoint) - mju_dot3(planeNormal,linePoint)) / mju_dot3(planeNormal,lineDirection);
    mju_addScl3(interPoint,linePoint,lineDirection,t);
    return true;
}