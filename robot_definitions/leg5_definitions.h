#ifndef LEG5_DEFINITIONS
#define LEG5_DEFINITIONS

/*------------------------------------------*/
/*------ Calibration Values for Leg 1 ------*/
/*------------------------------------------*/
#define LEG5_LEG_A_CALIB_VAL       262954497LL
#define LEG5_LEG_B_CALIB_VAL       172643656LL

#define LEG5_LEG_A_RAD_PER_CNT   9.8039216e-09
#define LEG5_LEG_B_RAD_PER_CNT   9.8039216e-09

#define LEG5_TRAN_A_CALIB_VAL      198005168LL
#define LEG5_TRAN_B_CALIB_VAL      197540925LL

#define LEG5_TRAN_A_RAD_PER_CNT  9.8039216e-09
#define LEG5_TRAN_B_RAD_PER_CNT -9.8039216e-09

#define LEG5_MOTOR_A_DIRECTION            -1.0
#define LEG5_MOTOR_B_DIRECTION             1.0




// Ignore the lines below, they are used for the include magic.
#ifdef INCLUDE_LEFT_LEG
#define LEFT_LEG_A_CALIB_VAL     LEG5_LEG_A_CALIB_VAL
#define LEFT_LEG_B_CALIB_VAL     LEG5_LEG_B_CALIB_VAL

#define LEFT_LEG_A_RAD_PER_CNT   LEG5_LEG_A_RAD_PER_CNT
#define LEFT_LEG_B_RAD_PER_CNT   LEG5_LEG_B_RAD_PER_CNT

#define LEFT_TRAN_A_CALIB_VAL    LEG5_TRAN_A_CALIB_VAL
#define LEFT_TRAN_B_CALIB_VAL    LEG5_TRAN_B_CALIB_VAL 

#define LEFT_TRAN_A_RAD_PER_CNT  LEG5_TRAN_A_RAD_PER_CNT
#define LEFT_TRAN_B_RAD_PER_CNT  LEG5_TRAN_B_RAD_PER_CNT

#define LEFT_MOTOR_A_DIRECTION   LEG5_MOTOR_A_DIRECTION
#define LEFT_MOTOR_B_DIRECTION   LEG5_MOTOR_B_DIRECTION
#endif

#ifdef INCLUDE_RIGHT_LEG
#define RIGHT_LEG_A_CALIB_VAL    LEG5_LEG_A_CALIB_VAL
#define RIGHT_LEG_B_CALIB_VAL    LEG5_LEG_B_CALIB_VAL

#define RIGHT_LEG_A_RAD_PER_CNT  LEG5_LEG_A_RAD_PER_CNT
#define RIGHT_LEG_B_RAD_PER_CNT  LEG5_LEG_B_RAD_PER_CNT

#define RIGHT_TRAN_A_CALIB_VAL   LEG5_TRAN_A_CALIB_VAL
#define RIGHT_TRAN_B_CALIB_VAL   LEG5_TRAN_B_CALIB_VAL 

#define RIGHT_TRAN_A_RAD_PER_CNT LEG5_TRAN_A_RAD_PER_CNT
#define RIGHT_TRAN_B_RAD_PER_CNT LEG5_TRAN_B_RAD_PER_CNT

#define RIGHT_MOTOR_A_DIRECTION  LEG5_MOTOR_A_DIRECTION
#define RIGHT_MOTOR_B_DIRECTION  LEG5_MOTOR_B_DIRECTION
#endif

#endif //LEG5_DEFINITIONS
