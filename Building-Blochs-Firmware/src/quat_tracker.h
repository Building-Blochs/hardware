#pragma once
// Dependency-free quaternion rotation tracker.
// Tracks the net rotation applied to the Bloch sphere so that a
// minimal-path reset can be computed later.

#include <cmath>

#ifndef PI
#  ifdef M_PI
#    define PI M_PI
#  else
#    define PI 3.14159265358979323846
#  endif
#endif

struct QuatTracker
{
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    // Apply a rotation given as (axis[3], turns) where turns=1 is a full 360°.
    void applyRotation(const double axis[3], double turns)
    {
        double ax = axis[0], ay = axis[1], az = axis[2];
        double mag = sqrt(ax * ax + ay * ay + az * az);
        if (mag == 0.0) return;
        ax /= mag; ay /= mag; az /= mag;

        double angle = turns * 2.0 * PI;
        double s     = sin(angle / 2.0);
        double qw    = cos(angle / 2.0);
        double qx    = ax * s, qy = ay * s, qz = az * s;

        // Hamilton product: q_new * q_current
        double nw = qw * w - qx * x - qy * y - qz * z;
        double nx = qw * x + qx * w + qy * z - qz * y;
        double ny = qw * y - qx * z + qy * w + qz * x;
        double nz = qw * z + qx * y - qy * x + qz * w;

        double nmag = sqrt(nw*nw + nx*nx + ny*ny + nz*nz);
        w = nw / nmag; x = nx / nmag; y = ny / nmag; z = nz / nmag;
    }

    // Compute the single rotation (shortest path) that undoes all applied
    // rotations and resets to identity.  Resets the tracker to identity.
    void calculateReset(double outAxis[3], double &outTurns)
    {
        double inv_w = w, inv_x = -x, inv_y = -y, inv_z = -z;

        if (inv_w >  1.0) inv_w =  1.0;
        if (inv_w < -1.0) inv_w = -1.0;

        double angle = 2.0 * acos(inv_w);
        double s     = sqrt(1.0 - inv_w * inv_w);

        if (s < 0.001)
        {
            outAxis[0] = 1.0; outAxis[1] = 0.0; outAxis[2] = 0.0;
            outTurns   = 0.0;
        }
        else
        {
            outAxis[0] = inv_x / s;
            outAxis[1] = inv_y / s;
            outAxis[2] = inv_z / s;
            outTurns   = angle / (2.0 * PI);
        }

        // Shortest path: never rotate more than half a turn
        if (outTurns > 0.5)
        {
            outTurns      = 1.0 - outTurns;
            outAxis[0]    = -outAxis[0];
            outAxis[1]    = -outAxis[1];
            outAxis[2]    = -outAxis[2];
        }

        // Reset to identity
        w = 1.0; x = 0.0; y = 0.0; z = 0.0;
    }
};
