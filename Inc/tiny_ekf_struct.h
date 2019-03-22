/*
 * tiny_ekf_struct.h: common data structure for TinyEKF
 *
 * You should #include this file after using #define for N (states) and M
*  (observations)
 *
 * Copyright (C) 2016 Simon D. Levy
 *
 * MIT License
 */

typedef struct {

    int n;          /* number of state values */
    int m;          /* number of observables */

    float x[Nsta];    /* state vector */

    float P[Nsta][Nsta];  /* prediction error covariance */
    float Q[Nsta][Nsta];  /* process noise covariance */
    float R[Mobs][Mobs];  /* measurement error covariance */

    float G[Nsta][Mobs];  /* Kalman gain; a.k.a. K */

    float F[Nsta][Nsta];  /* Jacobian of process model */
    float H[Mobs][Nsta];  /* Jacobian of measurement model */

    float Ht[Nsta][Mobs]; /* transpose of measurement Jacobian */
    float Ft[Nsta][Nsta]; /* transpose of process Jacobian */
    float Pp[Nsta][Nsta]; /* P, post-prediction, pre-update */

    float fx[Nsta];   /* output of user defined f() state-transition function */
    float hx[Mobs];   /* output of user defined h() measurement function */

    /* temporary storage */
    float tmp0[Nsta][Nsta];
    float tmp1[Nsta][Mobs];
    float tmp2[Mobs][Nsta];
    float tmp3[Mobs][Mobs];
    float tmp4[Mobs][Mobs];
    float tmp5[Mobs]; 

} ekf_t;        
