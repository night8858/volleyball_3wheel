#ifndef TRACKING_H
#define TRACKING_H

#include "tracking.h"
#include "user_lib.h"

typedef struct 
{
    float mid_traget_HIKVISION_x;
    float mid_traget_HIKVISION_y;

    float mid_traget_USBCAM_x;
    float mid_traget_USBCAM_y;
    
    float sub_offset_x;
    float sub_offset_y;

    first_order_filter_type_t HIKVISION_x_filter;
    first_order_filter_type_t HIKVISION_y_filter;
    
    first_order_filter_type_t USBCAM_x_filter;
    first_order_filter_type_t USBCAM_y_filter;

} s_tracking_data_t;

typedef struct {
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} kalman1_state;


/* 2 Dimension */
typedef struct {
    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
    float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
    float r;        /* measure noise convariance */
    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
    float gain[2];  /* 2x1 */
} kalman2_state;  




void pc_data_check(void);
void tracking_init(s_tracking_data_t *tracking_data);
void kalman1_init(kalman1_state *state, float init_x, float init_p, float init_q, float init_r);
float kalman1_filter(kalman1_state *state, float z_measure);
void chassis_volleyball_track(void);
void keep_ball_in_center_track(void);

void kalman2_init(kalman2_state *state, const float *init_x, float (*init_p)[2]);
void pc_data_filter_init(void);
void pc_data_update(float hik_data_x , float hik_data_y , float hik_data_z);
void ACTION_chassis_serve_tracking(void);


#endif
