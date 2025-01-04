/* USER CODE BEGIN Includes */
#include "servo.h"
/* USER CODE END Includes */


/* USER CODE BEGIN 2 */

    servo_init();                           /* 初始化舵机 */

    /* 示例：设置机械臂位置 */
    // 用户角度，范围根据每个舵机定义
    // SERVO_ID_1 和 SERVO_ID_2: -90 到 +90
    // SERVO_ID_3 和 SERVO_ID_4: -135 到 +135
    // SERVO_ID_5: -90 到 +90
    /* 测试代码 */
    // 设置所有舵机到90度
    //int16_t test_angles[SERVO_MAX_NUM] = {0, 0, 0, 0, 0};
    //servo_set_arm_position(test_angles);


    /* USER CODE END 2 */