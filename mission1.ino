
#include <Servo.h>
Servo servoX;   //X：底座旋转舵机
Servo servoY;   //Y：后臂舵机
Servo servoZ;   //Z：前臂舵机
Servo servoG;   //G：夹爪舵机
/* =====================================================================
 *  1. 基础控制：摇杆
 * ===================================================================== */
const int joy1X = A0;
const int joy1Y = A1;
const int joy2X = A2;
const int joy2Y = A3;
// ========== 舵机限位 ==========
const int X_MIN = 0,  X_MAX = 180;
const int Y_MIN = 45, Y_MAX = 125;
const int Z_MIN = 60, Z_MAX = 130;
const int G_OPEN  = 20;
const int G_CLOSE = 100;
// 关节当前角度
int angX = 90, angY = 90, angZ = 90, angG = G_OPEN;
int lastX = 90, lastY = 90, lastZ = 90, lastG = G_OPEN;
int speedStep = 2;          // 摇杆每周期角度变化步长；H/L 指令修改它
// 限幅工具
int constrainAngle(int val, int minA, int maxA)
{
  if(val < minA) return minA;
  if(val > maxA) return maxA;
  return val;
}
// ==================== 摇杆控制 ====================
void joystickControl()
{
  int j1x = analogRead(joy1X);
  int j1y = analogRead(joy1Y);
  int j2x = analogRead(joy2X);
  int j2Y = analogRead(joy2Y);
  const int deadZone = 40;
  if(abs(j1x - 512) > deadZone)
  {
    if(j1x > 512) angX -= speedStep;
    else          angX += speedStep;
  }
  angX = constrainAngle(angX, X_MIN, X_MAX);
  if(abs(j1y - 512) > deadZone)
  {
    if(j1y > 512) angY += speedStep;
    else          angY -= speedStep;
  }
  angY = constrainAngle(angY, Y_MIN, Y_MAX);
  if(abs(j2x - 512) > deadZone)
  {
    if(j2x > 512) angZ += speedStep;
    else          angZ -= speedStep;
  }
  angZ = constrainAngle(angZ, Z_MIN, Z_MAX);
  if(abs(j2Y - 512) > deadZone)
  {
    if(j2Y > 512) angG += speedStep;
    else          angG -= speedStep;
  }
  angG = constrainAngle(angG, G_OPEN, G_CLOSE);
  // 只有角度改变才输出舵机PWM
  if(angX != lastX){ servoX.write(angX); lastX = angX; }
  if(angY != lastY){ servoY.write(angY); lastY = angY; }
  if(angZ != lastZ){ servoZ.write(angZ); lastZ = angZ; }
  if(angG != lastG){ servoG.write(angG); lastG = angG; }
}

// ==================== 串口指令解析 ====================
void serialCommandParse()
{
  if(!Serial.available()) return;
  String buf = Serial.readStringUntil('\n');
  buf.trim();
  buf.toUpperCase();
  Serial.print("收到指令：");
  Serial.println(buf);
  // ── 任务一单字符：固定指令通信 ──
  if(buf == "O"){ angG = G_OPEN;  servoG.write(angG); Serial.println("→爪子张开"); return; }
  if(buf == "S"){ angG = G_CLOSE; servoG.write(angG); Serial.println("→爪子闭合"); return; }
  if(buf == "H"){ speedStep++; if(speedStep>8)speedStep=8; Serial.print("→提速，步长："); Serial.println(speedStep); return; }
  if(buf == "L"){ speedStep--; if(speedStep<1)speedStep=1; Serial.print("→降速，步长："); Serial.println(speedStep); return; }

  // ── 任务一：多舵机协同 x10,y30,z20 ──
  if(buf.startsWith("X"))
  {
    int idxY = buf.indexOf(",Y");
    int idxZ = buf.indexOf(",Z");
    if(idxY == -1 || idxZ == -1){
      Serial.println("指令格式错误，示例 x90,y80,z90");
      return;
    }
    int setX = buf.substring(1, idxY).toInt();
    int setY = buf.substring(idxY+2, idxZ).toInt();
    int setZ = buf.substring(idxZ+2).toInt();
    setX = constrainAngle(setX, X_MIN, X_MAX);
    setY = constrainAngle(setY, Y_MIN, Y_MAX);
    setZ = constrainAngle(setZ, Z_MIN, Z_MAX);
    servoX.write(setX); servoY.write(setY); servoZ.write(setZ);
    angX=setX; angY=setY; angZ=setZ;
    lastX=setX; lastY=setY; lastZ=setZ;
    Serial.print("→同步设置 X:"); Serial.print(setX);
    Serial.print(" Y:"); Serial.print(setY);
    Serial.print(" Z:"); Serial.println(setZ);
  }
}
/* =====================================================================
 *  3. 固定指令依赖的主循环与初始化
 * ===================================================================== */
void setup()
{
  Serial.begin(9600);
  servoX.attach(9);
  servoY.attach(8);
  servoZ.attach(7);
  servoG.attach(6);
  servoX.write(angX);
  servoY.write(angY);
  servoZ.write(angZ);
  servoG.write(angG);
  delay(500);
  Serial.println("MeArm任务一+二+三就绪");
  Serial.println("指令：O开爪 S关爪 H提速 L降速");
  Serial.println("同步舵机示例：x90,y80,z70");
}
void loop()
{
  joystickControl();     // 摇杆实时控制
  serialCommandParse();  // 串口指令处理
  delay(20);
}

