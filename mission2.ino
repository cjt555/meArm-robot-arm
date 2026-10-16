#include <Servo.h>
Servo servoX;   //X：底座旋转舵机
Servo servoY;   //Y：后臂舵机
Servo servoZ;   //Z：前臂舵机
Servo servoG;   //G：夹爪舵机

/* ====================== 任务一：引脚与限位 ====================== */

const int joy1X = A0;
const int joy1Y = A1;
const int joy2X = A2;
const int joy2Y = A3;    // A3 控制夹爪；任务一开关量控制只用到 X/Y/Z 三轴

// 舵机限位（按实物标定，单位：度）
const int X_MIN = 0,  X_MAX = 180;
const int Y_MIN = 45, Y_MAX = 125;
const int Z_MIN = 60, Z_MAX = 130;
const int G_OPEN  = 20;
const int G_CLOSE = 100;

// 上电初始姿态，也是回中目标
const int HOME_X = 90;
const int HOME_Y = 90;
const int HOME_Z = 90;
const int HOME_G = G_OPEN;

// 关节当前角度
int angX = 90, angY = 90, angZ = 90, angG = G_OPEN;
int lastX = 90, lastY = 90, lastZ = 90, lastG = G_OPEN;

int speedStep = 2;       // 摇杆每周期角度变化步长，H/L 指令修改

// 限幅
int constrainAngle(int val, int minA, int maxA)
{
  if(val < minA) return minA;
  if(val > maxA) return maxA;
  return val;
}

// 摇杆实时控制：三轴跟随 + A3 控制夹爪
void joystickControl()
{
  int j1x = analogRead(joy1X);
  int j1y = analogRead(joy1Y);
  int j2x = analogRead(joy2X);
  int j2Y = analogRead(joy2Y);
  const int deadZone = 40;      // 摇杆中值 512，±40 内视为不动

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

  // 有变化才写，避免高频重发相同角度
  if(angX != lastX){ servoX.write(angX); lastX = angX; }
  if(angY != lastY){ servoY.write(angY); lastY = angY; }
  if(angZ != lastZ){ servoZ.write(angZ); lastZ = angZ; }
  if(angG != lastG){ servoG.write(angG); lastG = angG; }
}

/* ====================== 非阻塞移动 ======================
 * moveToStart() 设定目标后立即返回，moveUpdate() 每周期走一小步。
 * 不能用阻塞式 for+delay 实现：一次搬运要移动七段，阻塞期间
 * 串口收不到指令，M 回中按不动。
 * ======================================================= */

const int STEP_DEG  = 2;     // 每周期步进角度，2 度配合 20ms 周期 ≈ 100°/s
const int ARRIVE_MS = 250;   // 到位后保持时间，等舵机真正停稳再走下一步

int goalX = -1, goalY = -1, goalZ = -1, goalG = -1;
bool isMoving = false;
unsigned long arriveAt = 0;

// 设定目标并开始移动；g 传 -1 表示夹爪保持不动
void moveToStart(int x, int y, int z, int g)
{
  goalX = constrainAngle(x, X_MIN, X_MAX);
  goalY = constrainAngle(y, Y_MIN, Y_MAX);
  goalZ = constrainAngle(z, Z_MIN, Z_MAX);
  if(g >= 0) goalG = constrainAngle(g, G_OPEN, G_CLOSE);
  else       goalG = angG;

  isMoving = true;
  arriveAt = 0;
}

// 每周期调用；返回 true 表示已到位且保持时间已满
bool moveUpdate()
{
  if(!isMoving) return true;

  int dX = goalX - angX;
  int dY = goalY - angY;
  int dZ = goalZ - angZ;
  int dG = goalG - angG;

  // 四轴同时步进，距离长的后到
  if(dX >  STEP_DEG) angX += STEP_DEG;
  else if(dX < -STEP_DEG) angX -= STEP_DEG;
  else angX = goalX;

  if(dY >  STEP_DEG) angY += STEP_DEG;
  else if(dY < -STEP_DEG) angY -= STEP_DEG;
  else angY = goalY;

  if(dZ >  STEP_DEG) angZ += STEP_DEG;
  else if(dZ < -STEP_DEG) angZ -= STEP_DEG;
  else angZ = goalZ;

  if(dG >  STEP_DEG) angG += STEP_DEG;
  else if(dG < -STEP_DEG) angG -= STEP_DEG;
  else angG = goalG;

  if(angX != lastX){ servoX.write(angX); lastX = angX; }
  if(angY != lastY){ servoY.write(angY); lastY = angY; }
  if(angZ != lastZ){ servoZ.write(angZ); lastZ = angZ; }
  if(angG != lastG){ servoG.write(angG); lastG = angG; }

  if(angX == goalX && angY == goalY && angZ == goalZ && angG == goalG)
  {
    if(arriveAt == 0) arriveAt = millis();
    if(millis() - arriveAt >= ARRIVE_MS)
    {
      isMoving = false;
      return true;
    }
  }
  return false;
}

/* ====================== 任务二：A / B / C 搬运 ======================
 * 点位表：抓取点与放置点各三个角度（X 底座 / Y 后臂 / Z 前臂，Z 越大夹爪越低）
 *
 * ⚠ C 点 Y 若小于 Y_MIN 会被限位夹住，标定值失效。
 *   原代码 C 点 Y=30 < Y_MIN=45，这里暂改 50；实物确实需要 30 就先放宽 Y_MIN。
 * ==================================================================== */

const int A_GRAB_X = 40,  A_GRAB_Y = 80,  A_GRAB_Z = 90;
const int A_PUT_X  = 130, A_PUT_Y  = 75,  A_PUT_Z  = 95;
const int B_GRAB_X = 60,  B_GRAB_Y = 85,  B_GRAB_Z = 88;
const int B_PUT_X  = 20,  B_PUT_Y  = 82,  B_PUT_Z  = 92;
const int C_GRAB_X = 50,  C_GRAB_Y = 50,  C_GRAB_Z = 100;
const int C_PUT_X  = 110, C_PUT_Y  = 55,  C_PUT_Z  = 100;

// 平移时的抬臂偏置，加到 Z 上（Z 越大越抬不起来，故加正值即抬臂）
// Z + LIFT 超过 Z_MAX 会被限位夹住，失去抬高效果
const int LIFT = 15;

// 动作基元：A/B/C 流程相同，只是点位不同
enum StepKind : uint8_t {
  ST_GRAB_POINT_OPEN,    // 到抓取点，爪张开
  ST_GRAB_POINT_CLOSE,   // 停在抓取点，爪闭合
  ST_GRAB_UP,            // 抬起离开台面
  ST_PUT_UP,             // 平移到放置点上方
  ST_PUT_DOWN,           // 下降到放置点
  ST_PUT_OPEN,           // 停在放置点，爪张开
  ST_PUT_RETREAT         // 抬起离开
};

const uint8_t STEP_LIST[] = {
  ST_GRAB_POINT_OPEN, ST_GRAB_POINT_CLOSE, ST_GRAB_UP,
  ST_PUT_UP, ST_PUT_DOWN, ST_PUT_OPEN, ST_PUT_RETREAT
};
const uint8_t STEP_COUNT = sizeof(STEP_LIST) / sizeof(STEP_LIST[0]);

bool seqRunning  = false;
uint8_t seqObj   = 0;    // 0=A 1=B 2=C
uint8_t seqIndex = 0;

// 把第 i 步翻译成四轴目标并启动移动
void seqStartStep(uint8_t i)
{
  int gx, gy, gz, px, py, pz;

  if(seqObj == 0)      { gx = A_GRAB_X; gy = A_GRAB_Y; gz = A_GRAB_Z;
                       px = A_PUT_X;  py = A_PUT_Y;  pz = A_PUT_Z; }
  else if(seqObj == 1) { gx = B_GRAB_X; gy = B_GRAB_Y; gz = B_GRAB_Z;
                         px = B_PUT_X;  py = B_PUT_Y;  pz = B_PUT_Z; }
  else                 { gx = C_GRAB_X; gy = C_GRAB_Y; gz = C_GRAB_Z;
                         px = C_PUT_X;  py = C_PUT_Y;  pz = C_PUT_Z; }

  switch(STEP_LIST[i])
  {
    case ST_GRAB_POINT_OPEN:  moveToStart(gx, gy, gz, G_OPEN);        break;
    case ST_GRAB_POINT_CLOSE: moveToStart(gx, gy, gz, G_CLOSE);       break;
    case ST_GRAB_UP:          moveToStart(gx, gy, gz + LIFT, G_CLOSE);break;
    case ST_PUT_UP:           moveToStart(px, py, pz + LIFT, G_CLOSE);break;
    case ST_PUT_DOWN:         moveToStart(px, py, pz, G_CLOSE);       break;
    case ST_PUT_OPEN:         moveToStart(px, py, pz, G_OPEN);        break;
    case ST_PUT_RETREAT:      moveToStart(px, py, pz + LIFT, G_OPEN); break;
  }
}

// 启动一次搬运。obj：0=A 1=B 2=C
void runTask2(uint8_t obj)
{
  if(seqRunning)
  {
    Serial.println("→ 正在搬运中，先按 M 回中止");
    return;
  }

  seqObj     = obj;
  seqIndex   = 0;
  seqRunning = true;

  Serial.print("==== 开始夹取 ");
  Serial.print(obj == 0 ? "A" : (obj == 1) ? "B" : "C");
  Serial.println(" 物体 ====");

  seqStartStep(0);
}

// 任务二推进，每周期调用一次
void seqUpdate()
{
  if(!seqRunning) return;
  if(!moveUpdate()) return;

  seqIndex++;
  if(seqIndex >= STEP_COUNT)
  {
    seqRunning = false;
    Serial.print("==== ");
    Serial.print(seqObj == 0 ? "A" : (seqObj == 1) ? "B" : "C");
    Serial.println(" 搬运完成 ====");
    return;
  }
  seqStartStep(seqIndex);
}

// 回中；同时中断正在进行的搬运
void goHome()
{
  Serial.println("→ 回中");
  seqRunning = false;
  moveToStart(HOME_X, HOME_Y, HOME_Z, HOME_G);
}

/* ====================== 串口指令解析 ====================== */

void printHelp()
{
  Serial.println("O 开爪  S 关爪  H 提速  L 降速");
  Serial.println("x90,y80,z70  同步设角度");
  Serial.println("A / B / C    自动夹取搬运（任务二）");
  Serial.println("T40,80,90  平滑移动   M 回中   Q 状态   ? 帮助");
}

void printStatus()
{
  Serial.print("状态：");
  if(seqRunning)    Serial.print("搬运中");
  else if(isMoving) Serial.print("移动中");
  else              Serial.print("待机");
  Serial.print("  步长："); Serial.print(speedStep);
  Serial.print("  X:"); Serial.print(angX);
  Serial.print(" Y:");    Serial.print(angY);
  Serial.print(" Z:");    Serial.print(angZ);
  Serial.print(" G:");    Serial.println(angG);
}

void serialCommandParse()
{
  if(!Serial.available()) return;
  String buf = Serial.readStringUntil('\n');
  buf.trim();
  buf.toUpperCase();

  Serial.print("收到指令：");
  Serial.println(buf);

  // 任务一单字符
  if(buf == "O"){ angG = G_OPEN;  servoG.write(angG); lastG = angG; Serial.println("→爪子张开"); return; }
  if(buf == "S"){ angG = G_CLOSE; servoG.write(angG); lastG = angG; Serial.println("→爪子闭合"); return; }
  if(buf == "H"){ speedStep++; if(speedStep>8)speedStep=8; Serial.print("→提速，步长："); Serial.println(speedStep); return; }
  if(buf == "L"){ speedStep--; if(speedStep<1)speedStep=1; Serial.print("→降速，步长："); Serial.println(speedStep); return; }

  // M 回中是软急停，必须放在忙碌判断之前
  if(buf == "M"){ goHome(); return; }
  if(buf == "Q"){ printStatus(); return; }
  if(buf == "?"){ printHelp();   return; }

  if(seqRunning || isMoving)
  {
    Serial.println("→ 忙碌中（按 M 可立即中止）");
    return;
  }

  // 任务二
  if(buf == "A"){ runTask2(0); return; }
  if(buf == "B"){ runTask2(1); return; }
  if(buf == "C"){ runTask2(2); return; }

  // 任务一：多舵机协同
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
    return;
  }

  // 标定用：平滑移动到指定位姿
  if(buf.startsWith("T"))
  {
    int idx1 = buf.indexOf(",");
    int idx2 = buf.indexOf(",", idx1+1);
    if(idx1 == -1 || idx2 == -1){
      Serial.println("格式错误，示例 T40,80,90");
      return;
    }
    int tx = buf.substring(1, idx1).toInt();
    int ty = buf.substring(idx1+1, idx2).toInt();
    int tz = buf.substring(idx2+1).toInt();
    moveToStart(tx, ty, tz, -1);
    Serial.println("→ 平滑移动中");
    return;
  }

  Serial.println("→ 未知指令，发 ? 看帮助");
}

/* ====================== setup / loop ====================== */

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

  Serial.println("MeArm 任务一 + 任务二 就绪");
  printHelp();
}

void loop()
{
  serialCommandParse();                 // 先收串口，保证 M 急停最及时

  if(seqRunning) seqUpdate();
  else           moveUpdate();

  if(!seqRunning && !isMoving)          // 待机时才响应摇杆
    joystickControl();

  delay(20);
}