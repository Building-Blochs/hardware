#pragma once
// Pure Rodrigues rotation and motor-matrix math.
// No Arduino / hardware dependencies — safe to include in native tests.

#include <cmath>

#ifndef PI
#  ifdef M_PI
#    define PI M_PI
#  else
#    define PI 3.14159265358979323846
#  endif
#endif

// ---------------------------------------------------------------------------
// Rodrigues rotation:  rotate vector v around unit axis k by angle theta (rad)
// Both v and k are plain double[3].
// ---------------------------------------------------------------------------
inline void rodrigues_rotate(
    const double v[3],
    const double k_unnorm[3],
    double       theta,
    double       out[3])
{
    // normalise k
    double kx = k_unnorm[0], ky = k_unnorm[1], kz = k_unnorm[2];
    double kmag = sqrt(kx*kx + ky*ky + kz*kz);
    if (kmag == 0.0) { out[0]=v[0]; out[1]=v[1]; out[2]=v[2]; return; }
    kx /= kmag; ky /= kmag; kz /= kmag;

    double c = cos(theta);
    double s = sin(theta);
    double dot = kx*v[0] + ky*v[1] + kz*v[2];

    // cross(k, v)
    double cx = ky*v[2] - kz*v[1];
    double cy = kz*v[0] - kx*v[2];
    double cz = kx*v[1] - ky*v[0];

    out[0] = v[0]*c + cx*s + kx*dot*(1.0 - c);
    out[1] = v[1]*c + cy*s + ky*dot*(1.0 - c);
    out[2] = v[2]*c + cz*s + kz*dot*(1.0 - c);
}

// Normalise a 3-vector in-place.
inline void vec3_normalize(double v[3])
{
    double mag = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag == 0.0) return;
    v[0] /= mag; v[1] /= mag; v[2] /= mag;
}

// Dot product of two 3-vectors.
inline double vec3_dot(const double a[3], const double b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// ---------------------------------------------------------------------------
// Motor matrix: maps a rotation axis (in Bloch-sphere coords) to motor steps.
// wa_deg  = wheel mount angle from vertical (MOTOR_ANGLE_DEG)
// zr_deg  = base Z rotation offset (BASE_Z_ROTATION_DEG)
// out[3][3] stored row-major, same convention as Eigen Matrix3d.
// ---------------------------------------------------------------------------
inline void generate_motor_matrix(double wa_deg, double zr_deg, double out[3][3])
{
    double wa  = wa_deg  * PI / 180.0;
    double zr  = zr_deg  * PI / 180.0;

    // Wheel position vectors (radial direction from centre of ball to wheel contact)
    double wv[3][3], wn[3][3], wd[3][3], wm[3][3];
    for (int i = 0; i < 3; ++i)
    {
        double phi = i * 2.0 * PI / 3.0 + zr;
        wv[i][0] =  sin(wa) * cos(phi);
        wv[i][1] =  sin(wa) * sin(phi);
        wv[i][2] = -cos(wa);

        wn[i][0] =  cos(wa) * cos(phi);
        wn[i][1] =  cos(wa) * sin(phi);
        wn[i][2] =  sin(wa);

        // wheel drive direction = wv x wn
        wd[i][0] = wv[i][1]*wn[i][2] - wv[i][2]*wn[i][1];
        wd[i][1] = wv[i][2]*wn[i][0] - wv[i][0]*wn[i][2];
        wd[i][2] = wv[i][0]*wn[i][1] - wv[i][1]*wn[i][0];

        // motion direction = wv x wd, then normalise
        wm[i][0] = wv[i][1]*wd[i][2] - wv[i][2]*wd[i][1];
        wm[i][1] = wv[i][2]*wd[i][0] - wv[i][0]*wd[i][2];
        wm[i][2] = wv[i][0]*wd[i][1] - wv[i][1]*wd[i][0];
        double mag = sqrt(wm[i][0]*wm[i][0] + wm[i][1]*wm[i][1] + wm[i][2]*wm[i][2]);
        if (mag > 0) { wm[i][0]/=mag; wm[i][1]/=mag; wm[i][2]/=mag; }
    }

    // Final sign flip pattern matches bloch.cpp::generateMotorMatrix
    for (int i = 0; i < 3; ++i)
    {
        out[i][0] = -wm[i][0];
        out[i][1] =  wm[i][1];
        out[i][2] = -wm[i][2];
    }
}
