#ifndef CODE_PWM3901_H_
#define CODE_PWM3901_H_

//
//#define PMW3901_CSCALE          (0.001f)          // deg/pixel，需要实测标定
//#define DEG_TO_RAD              (3.14159265358979f / 180.0f)
//#define PMW3901_PERIOD          (0.02f)           // 20ms 采样周期

//#define scale           0         // 20ms 采样周期
//#define K               0       // 20ms 采样周期
//
//
void OF_init(void);
void OF_data_deal(float dt);
void velocity_mahony_fusion(float dt);

#endif 
