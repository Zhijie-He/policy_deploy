//
// Created by willw on 24/10/23.
//

#pragma once

#include "Eigen/Dense"

/***************************************************************************/
//reading xbox 360 wireless joystick's instructions
#define XBOX_TYPE_BUTTON    0x01
#define XBOX_TYPE_AXIS      0x02
#define XBOX_BUTTON_A       0x00
#define XBOX_BUTTON_B       0x01
#define XBOX_BUTTON_X       0x03
#define XBOX_BUTTON_Y       0x04
#define XBOX_BUTTON_LB      0x06
#define XBOX_BUTTON_RB      0x07
#define XBOX_BUTTON_START   0x0B
#define XBOX_BUTTON_BACK    0x0A
#define XBOX_BUTTON_HOME    0x0C
#define XBOX_BUTTON_LO      0x0E    /* 左摇杆按键 */
#define XBOX_BUTTON_RO      0x0F    /* 右摇杆按键 */
#define XBOX_BUTTON_ON      0x01
#define XBOX_BUTTON_OFF     0x00

#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */
#define XBOX_AXIS_RX        0x02    /* 右摇杆X轴 */
#define XBOX_AXIS_RY        0x03    /* 右摇杆Y轴 */
#define XBOX_AXIS_LT        0x04
#define XBOX_AXIS_RT        0x05
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP        (-32767)
#define XBOX_AXIS_VAL_DOWN      32767
#define XBOX_AXIS_VAL_LEFT      (-32767)
#define XBOX_AXIS_VAL_RIGHT     32767

#define XBOX_AXIS_VAL_MIN       (-32767)
#define XBOX_AXIS_VAL_MAX       32767
#define XBOX_AXIS_VAL_MID       0x00
typedef struct
{
    bool isAlive;
    int     time;
    int     a;
    int     b;
    int     x;
    int     y;
    int     lb;
    int     rb;
    int     start;
    int     back;
    int     home;
    int     lo;
    int     ro;

    int     lx;
    int     ly;
    int     rx;
    int     ry;
    int     lt;
    int     rt;
    int     xx;
    int     yy;

}xbox_map_t;

typedef struct JoystickData {
  bool isAlive;
  Eigen::VectorXf JoyLeft;
  Eigen::VectorXf JoyRight;
  Eigen::VectorXf pad;
  double triggerLeft;
  double triggerRight;
  bool buttonA;
  bool buttonB;
  bool buttonX;
  bool buttonY;
  bool buttonStart;
  bool shoulderLeft;
  bool shoulderRight;
} JoystickData;

inline JoystickData zeroJoystick() {
  JoystickData ret;
  ret.JoyLeft.setZero(2);
  ret.JoyRight.setZero(2);
  ret.pad.setZero(2);
  ret.triggerLeft = 0;
  ret.triggerRight = 0;
  ret.isAlive = false;
  ret.buttonStart = false;
  ret.buttonA = false;
  ret.buttonB = false;
  ret.buttonX = false;
  ret.buttonY = false;
  return ret;
}
