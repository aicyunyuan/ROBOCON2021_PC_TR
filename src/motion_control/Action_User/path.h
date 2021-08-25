#ifndef _PATH_H
#define _PATH_H

#include "MotionCard.h"
#include "stdint.h"

#define PULSE_2_VEL(pulse) (((float)pulse/MOTOR_PULSE_PER_ROUND)/WHEEL_RATIO*PI*WHEEL_DIAMETER)
#define MAX_PLAN_VEL PULSE_2_VEL(MAX_MOTOR_SPEED)
#define MAX_TAR_VEL (10000.f)

//测试轨迹长度
#define ATTACK_PATH_1_NUM_DEF (76)
#define ATTACK_PATH_2_NUM_DEF (25)
#define ATTACK_PATH_3_NUM_DEF (33)
#define ATTACK_PATH_4_NUM_DEF (30)
#define ATTACK_PATH_5_NUM_DEF (37)
#define ATTACK_PATH_6_NUM_DEF (20)
#define ATTACK_PATH_7_NUM_DEF (20)
#define ATTACK_PATH_8_NUM_DEF (58)
#define ATTACK_PATH_9_NUM_DEF (42)
#define ATTACK_PATH_10_NUM_DEF (42)
#define ATTACK_PATH_11_NUM_DEF (63)
#define ATTACK_PATH_TAKE_NUM_DEF (12)
#define ATTACK_PATH_TAKE_AGAIN_NUM_DEF (9)
#define ATTACK_PATH_ARCHERY_FIRST_NUM_DEF (15)
#define ATTACK_PATH_ARCHERY_SEC_NUM_DEF (15)

#define TEST_PATH_NUM_DEF (30)

extern uint8_t ATTACK_PATH_1_NUM;
extern uint8_t ATTACK_PATH_2_NUM;
extern uint8_t ATTACK_PATH_3_NUM;
extern uint8_t ATTACK_PATH_4_NUM;
extern uint8_t ATTACK_PATH_5_NUM;
extern uint8_t ATTACK_PATH_6_NUM;
extern uint8_t ATTACK_PATH_7_NUM;
extern uint8_t ATTACK_PATH_8_NUM;
extern uint8_t ATTACK_PATH_9_NUM;
extern uint8_t ATTACK_PATH_10_NUM;
extern uint8_t ATTACK_PATH_11_NUM;
extern uint8_t ATTACK_PATH_TAKE_NUM;
extern uint8_t ATTACK_PATH_TAKE_AGAIN_NUM;
extern uint8_t ATTACK_PATH_ARCHERY_FIRST_NUM;
extern uint8_t ATTACK_PATH_ARCHERY_SEC_NUM;

extern uint8_t TEST_PATH_NUM;

extern Pose_t AttackPath1[];
extern Pose_t AttackPath2[];
extern Pose_t AttackPath3[];
extern Pose_t AttackPath4[];
extern Pose_t AttackPath5[];
extern Pose_t AttackPath6[];
extern Pose_t AttackPath7[];
extern Pose_t AttackPath8[];
extern Pose_t AttackPath9[];
extern Pose_t AttackPath10[];
extern Pose_t AttackPath11[];
extern Pose_t AttackPathTake[];
extern Pose_t AttackPathTakeAgain[];
extern Pose_t AttackPathArcheryFirst[];
extern Pose_t AttackPathArcherySec[];

extern Pose_t testPath[];

#endif


