/* aplqr.c */
#include "aplqr.h"
#include "pss.h"
#include <math.h>

static const float Ix = 3.3333e-3f, Iy = 1.01793e-2f, Iz = 1.01793e-2f; /* your inertias */
static const float Rinv[3]  = { 2.5e-9f, 2.5e-9f, 2.5e-9f };  /* design m_max^2 (5e-5)^2 */
static const float M_MAX_HW = 0.166f;                          /* hardware saturation */

void aplqr_step(const adcs_state_t *x, const float B_lvlh[3], adcs_dipole_t *m_cmd){
    const float Bx=B_lvlh[0], By=B_lvlh[1], Bz=B_lvlh[2];

    /* B_tv (6x3) — only rate-derivative rows nonzero, signs match make_B_tv() */
    float Btv[6][3] = {{0}};
    Btv[1][0]=0.f;      Btv[1][1]=Bz/Ix;   Btv[1][2]=-By/Ix;
    Btv[3][0]=-Bz/Iy;   Btv[3][1]=0.f;     Btv[3][2]=Bx/Iy;
    Btv[5][0]=By/Iz;    Btv[5][1]=-Bx/Iz;  Btv[5][2]=0.f;

    /* K = Rinv * Btv' * Pss   (3x6) */
    float K[3][6];
    for (int i=0;i<3;i++)
        for (int j=0;j<6;j++) {
            float acc=0.f;
            for (int r=0;r<6;r++) acc += Btv[r][i]*Pss[r][j];
            K[i][j]=Rinv[i]*acc;
        }

    const float xv[6]={x->phi,x->phi_dot,x->theta,x->theta_dot,x->psi,x->psi_dot};

    float u[3];
    for (int i=0;i<3;i++){ float acc=0.f; for(int j=0;j<6;j++) acc+=K[i][j]*xv[j]; u[i]=-acc; }

    /* direction-preserving saturation — same as your beta-scaling */
    float beta=0.f;
    for (int i=0;i<3;i++){ float a=fabsf(u[i])/M_MAX_HW; if(a>beta) beta=a; }
    float s = (beta>1.f)? 1.f/beta : 1.f;
    m_cmd->mx=u[0]*s; m_cmd->my=u[1]*s; m_cmd->mz=u[2]*s;
}

//void torquer_apply(const adcs_dipole_t *m)
//{
//    float duty[3] = { m->mx/k_torquer[0], m->my/k_torquer[1], m->mz/k_torquer[2] };
//    for (int i=0;i<3;i++){
//        if (duty[i]> 1.f) duty[i]= 1.f;
//        if (duty[i]<-1.f) duty[i]=-1.f;
//        drv8833_set_axis(i, duty[i]);  /* sign -> bridge direction, |duty| -> CCR */
//    }
//}
//
//void adcs_task(sat_mode_t mode)
//{
//    TickType_t next = xTaskGetTickCount();
//    const TickType_t Ts = pdMS_TO_TICKS(100);   /* 10 Hz, == dt_ctrl */
//
//    for (;;) {
//        adcs_state_t x;
//
//        float B_lvlh[3];
//        adcs_get_estimate(&x, B_lvlh);          /* from attitude determination */
//
//        if (eps_soc_low()) mode = MODE_SAFE;     /* global SoC gate overrides all */
//
//        adcs_dipole_t m = {0,0,0};
//        switch (mode) {
//            case MODE_DETUMBLE:
//                bdot_step(B_lvlh, &m); torquer_apply(&m); break;
//
//            case MODE_POINTING:
//                if (within_capture(&x))           /* APLQR is fine-pointing only */
//                    aplqr_step(&x, B_lvlh, &m);
//                else
//                    coarse_point_step(&x, B_lvlh, &m);
//                torquer_apply(&m); break;
//
//            case MODE_SCIENCE:
//                if (science_in_sample_window())   /* Langmuir needs torquers OFF */
//                    torquer_off();
//                else { aplqr_step(&x, B_lvlh, &m); torquer_apply(&m); }
//                break;
//
//            case MODE_SAFE: case MODE_LAUNCH: default:
//                torquer_off(); break;
//        }
//
//        IWDG_Refresh(&hiwdg);                     /* kick only on a healthy cycle */
//        vTaskDelayUntil(&next, Ts);
//    }
//}
//
//
