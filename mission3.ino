#include <Servo.h>

Servo servoX;   //X：底座旋转舵机
Servo servoY;   //Y：后臂舵机
Servo servoZ;   //Z：前臂舵机（角度越大夹爪越低）
Servo servoG;   //G：夹爪舵机

/* ====================== 一、引脚定义 ====================== */

const int joy1X = A0;    // 摇杆1 水平 → X 底座
const int joy1Y = A1;    // 摇杆1 垂直 → Y 后臂
const int joy2X = A2;    // 摇杆2 水平 → Z 前臂
const int joy2Y = A3;    // 摇杆2 垂直 → G 夹爪

/* 任务三四个按键。数组下标 = 按键编号 0~3，和 keyAction() 一一对应 */
const int keyPin1 = 2;   // 按键1：循环执行 A→B→C
const int keyPin2 = 3;   // 按键2：录制起止
const int keyPin3 = 4;   // 按键3：播放
const int keyPin4 = 5;   // 按键4：回中
const int keyPins[4] = { keyPin1, keyPin2, keyPin3, keyPin4 };

/* ====================== 二、限位、姿态与全局状态 ====================== */

// 舵机限位（单位：度），必须按实物标定
const int X_MIN = 0,  X_MAX = 180;
const int Y_MIN = 45, Y_MAX = 125;
const int Z_MIN = 60, Z_MAX = 130;
const int G_OPEN  = 20;      // 夹爪完全张开
const int G_CLOSE = 100;     // 夹爪完全闭合

// 上电初始姿态，也是回中目标
const int HOME_X = 90;
const int HOME_Y = 90;
const int HOME_Z = 90;
const int HOME_G = G_OPEN;

/* 关节当前角度。lastX~lastG 是上次真正下发的值，
 * 用来实现"有变化才写舵机"，避免静止时反复重发同样角度。
 * 不在声明处赋初值（放到 setup），让它们落在 .bss 段省 Flash。 */
int angX, angY, angZ, angG;
int lastX, lastY, lastZ, lastG;

int speedStep;          // 摇杆每周期角度步长，H/L 指令修改，范围 1~8

/* 工作模式：手动 / 录制 / 播放。
 * 任务二搬运用单独的 seqRunning 标志，因为它和这三个状态是不同维度
 * （搬运过程中仍允许按键4 回中）。
 * ★ 必须在 goHome() 之前定义：goHome 要清录制/播放状态。 */
enum RecMode : uint8_t { MODE_MANUAL, MODE_RECORD, MODE_PLAY };
RecMode recordMode;

// 限幅
int constrainAngle(int val, int minA, int maxA)
{
  if(val < minA) return minA;
  if(val > maxA) return maxA;
  return val;
}

/* ====================== 三、任务一：摇杆实时控制 ====================== */

/* 摇杆映射：摇杆1-X→底座 摇杆1-Y→后臂 摇杆2-X→前臂 摇杆2-Y→夹爪 */
void joystickControl()
{
  int j1x = analogRead(joy1X);   // ADC 读数 0~1023，中值约 512
  int j1y = analogRead(joy1Y);
  int j2x = analogRead(joy2X);
  int j2Y = analogRead(joy2Y);

  const int deadZone = 40;       // 摇杆中值 ±40 内视为不动

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

  /* 夹爪限位用 G_OPEN/G_CLOSE 而不是 0~180，
   * 这样摇杆能连续调开合程度，适配不同粗细的物体 */
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

/* ====================== 四、非阻塞移动 ======================
 * moveToStart() 设定目标后立即返回，moveUpdate() 每周期走一小步。
 * 不能用阻塞式 for+delay：一次搬运要移动七段，阻塞期间
 * 串口收不到指令，回中急停也按不动。
 * ========================================================= */

const int STEP_DEG = 2;      // 每周期步进角度，2 度配合 20ms 周期 ≈ 100°/s
const int ARRIVE_MS = 250;   // 到位后保持时间。物体在抬臂时掉落就加大它，
                             // 本质是给夹爪留出真正夹紧的时间（防掉件）

int goalX, goalY, goalZ, goalG;   // 移动目标
bool isMoving;                    // 是否正在移动
unsigned long arriveAt;           // 四轴到位时刻，0 = 还没开始计时

// 设定目标并开始移动；g 传负数表示夹爪保持不动
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

  /* 四轴同时步进。剩余距离不足一步时直接对齐目标，
   * 否则会冲过目标再往回走，形成来回抖动。 */
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
    /* 用 arriveAt == 0 判断"是否已记过时刻"，
     * 否则每周期都刷新时间戳，保持时间永远等不满 */
    if(arriveAt == 0) arriveAt = millis();

    if(millis() - arriveAt >= ARRIVE_MS)
    {
      isMoving = false;
      return true;
    }
  }
  return false;
}

/* ====================== 五、任务二：A / B / C 搬运 ======================
 * ⚠ C 点的 Y 若小于 Y_MIN 会被限位夹住，标定值失效。
 *   原代码 C 点 Y=30 < Y_MIN=45，这里暂改 50；
 *   实物确实需要 30 就先放宽 Y_MIN。
 * ==================================================================== */

const int A_GRAB_X = 40,  A_GRAB_Y = 80,  A_GRAB_Z = 90;   // A 抓取点
const int A_PUT_X  = 130, A_PUT_Y  = 75,  A_PUT_Z  = 95;   // A 放置点
const int B_GRAB_X = 60,  B_GRAB_Y = 85,  B_GRAB_Z = 88;   // B 抓取点
const int B_PUT_X  = 20,  B_PUT_Y  = 82,  B_PUT_Z  = 92;   // B 放置点
const int C_GRAB_X = 50,  C_GRAB_Y = 50,  C_GRAB_Z = 100;  // C 抓取点
const int C_PUT_X  = 110, C_PUT_Y  = 55,  C_PUT_Z  = 100;  // C 放置点

/* 抬臂偏置，加到 Z 上。前臂 Z 越大夹爪越低，所以加正值即抬臂，
 * 用于平移时离开台面。
 * ⚠ Z + LIFT 超过 Z_MAX 会被限位夹住，结果就是抬不起来、照样蹭台面 */
const int LIFT = 15;

/* 动作基元。A/B/C 流程相同（到点→夹→抬→移→放→松→抬），只是点位不同 */
enum StepKind : uint8_t {
  ST_GRAB_POINT_OPEN,    // 到抓取点，爪张开
  ST_GRAB_POINT_CLOSE,   // 原地闭合，夹住
  ST_GRAB_UP,            // 抬高离开台面
  ST_PUT_UP,             // 平移到放置点上方
  ST_PUT_DOWN,           // 下降到放置点
  ST_PUT_OPEN,           // 原地张开，放开
  ST_PUT_RETREAT         // 抬起离开
};

const uint8_t STEP_LIST[] = {
  ST_GRAB_POINT_OPEN, ST_GRAB_POINT_CLOSE, ST_GRAB_UP,
  ST_PUT_UP, ST_PUT_DOWN, ST_PUT_OPEN, ST_PUT_RETREAT
};
const uint8_t STEP_COUNT = sizeof(STEP_LIST) / sizeof(STEP_LIST[0]);

bool seqRunning;         // 是否正在搬运
uint8_t seqObj;          // 本次搬哪个：0=A 1=B 2=C
uint8_t seqIndex;        // 当前第几步

/* 按键1 的循环游标，与 seqObj 分开：
 * seqObj 是"正在搬的物体"（搬运中不能变），
 * key1Index 是"下次该搬哪个"，按一次前进一格 */
uint8_t key1Index;

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
    case ST_GRAB_POINT_OPEN:  moveToStart(gx, gy, gz, G_OPEN);         break;
    case ST_GRAB_POINT_CLOSE: moveToStart(gx, gy, gz, G_CLOSE);        break;
    case ST_GRAB_UP:          moveToStart(gx, gy, gz + LIFT, G_CLOSE); break;
    case ST_PUT_UP:           moveToStart(px, py, pz + LIFT, G_CLOSE); break;
    case ST_PUT_DOWN:         moveToStart(px, py, pz, G_CLOSE);        break;
    case ST_PUT_OPEN:         moveToStart(px, py, pz, G_OPEN);         break;
    case ST_PUT_RETREAT:      moveToStart(px, py, pz + LIFT, G_OPEN);  break;
  }
}

// 启动一次搬运。obj：0=A 1=B 2=C
void runTask2(uint8_t obj)
{
  if(seqRunning)
  {
    Serial.println(F("→ 正在搬运中，按按键4 回中止"));
    return;
  }

  seqObj     = obj;
  seqIndex   = 0;
  seqRunning = true;

  Serial.print(F("==== 开始夹取 "));
  Serial.print(obj == 0 ? F("A") : (obj == 1) ? F("B") : F("C"));
  Serial.println(F(" 物体 ===="));

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
    Serial.print(F("==== "));
    Serial.print(seqObj == 0 ? F("A") : (seqObj == 1) ? F("B") : F("C"));
    Serial.println(F(" 搬运完成 ===="));
    return;
  }
  seqStartStep(seqIndex);
}

/* 按键1：循环执行 A→B→C→A。
 * 先启动搬运再推进游标，顺序不能反，否则第一次按会跳过 A */
void cycleTask2()
{
  runTask2(key1Index);
  key1Index = (key1Index + 1) % 3;
}

/* 回中，同时中断任务二搬运和任务三录制/播放（软急停）。
 * ★ 串口解析里必须放在"忙碌判断"之前，按键4 也必须无条件响应，
 *   否则搬运或录制过程中就没有办法停下来了。 */
void goHome()
{
  Serial.println(F("→ 回中"));

  /* 先清流程状态。若只改目标而不清 seqRunning，
   * 下一步 seqUpdate 会立刻用搬运目标覆盖回中目标，表现为"回中没反应" */
  seqRunning = false;
  recordMode = MODE_MANUAL;

  moveToStart(HOME_X, HOME_Y, HOME_Z, HOME_G);
}

/* ====================== 六、任务三：录制 / 播放 ======================
 *  ── 录制缓存的内存账（★ 别随意改大）──────────────────────────────
 *    ATmega328P 只有 2048 字节 SRAM，本程序静态占用 995 字节，
 *    所以录制缓存最多只能占 600~700 字节，否则一进录制就爆栈
 *    （表现为随机重启、动作错乱）。
 * ==================================================================== */

const int REC_INTERVAL = 60;   // 采样周期 ms，同时是回放节奏
const int REC_MAX = 200;       // 最多点数。★ 改大会直接吃掉栈空间

/* 录制缓存，每点 3 字节。位布局（每轴 6 bit，共 24 bit）：
 *   p[0] = aaaaaabb      a 的高 6 位 + b 的高 2 位
 *   p[1] = bbbbcccc      b 的低 4 位 + c 的高 4 位
 *   p[2] = ccdddddd      c 的低 2 位 + d 的 6 位 */
uint8_t recBuf[REC_MAX * 3];

int  recCount;                  // 已录点数
int  playIndex;                 // 回放进度
unsigned long recLastMs;        // 上次采样时刻
unsigned long playLastMs;       // 上次推进时刻

/* 角度 → 6 bit。用 (angle+2)/4 是四舍五入到最近 4°；
 * 直接整除会一直向下取整，误差单向累积到 3° */
uint8_t packAngle(int angle)
{
  int v = (angle + 2) / 4;
  if(v < 0)  v = 0;
  if(v > 63) v = 63;
  return (uint8_t)v;
}

// 把当前姿态打包写进第 i 个点
void writeRecord(int i)
{
  uint8_t *p = &recBuf[i * 3];
  uint8_t a = packAngle(angX);
  uint8_t b = packAngle(angY);
  uint8_t c = packAngle(angZ);
  uint8_t d = packAngle(angG);

  p[0] = (uint8_t)((a << 2) | (b >> 4));
  p[1] = (uint8_t)(((b & 0x0F) << 4) | (c >> 2));
  p[2] = (uint8_t)(((c & 0x03) << 6) | d);
}

// 解出第 i 个点的角度到 out[0..3]（对应 X Y Z G）
void readRecord(int i, int *out)
{
  const uint8_t *p = &recBuf[i * 3];
  out[0] = ((p[0] >> 2) & 0x3F) * 4;
  out[1] = (((p[0] & 0x03) << 4) | (p[1] >> 4)) * 4;
  out[2] = (((p[1] & 0x0F) << 2) | (p[2] >> 6)) * 4;
  out[3] = (p[2] & 0x3F) * 4;
}

/* 按键2：录制起止。同一函数靠 recordMode 判断当前状态，
 * 板载按键和串口 R 指令共用这份逻辑 */
void toggleRecord()
{
  if(recordMode == MODE_RECORD)         // 第二次按：结束
  {
    recordMode = MODE_MANUAL;

    if(recCount < 2)                    // 按下去就松开，没有有效动作
    {
      recCount = 0;
      Serial.println(F("→ 样本太少，已放弃"));
      return;
    }

    Serial.print(F("→ 录制结束，共 "));
    Serial.print(recCount);
    Serial.print(F(" 点，约 "));
    Serial.print((long)recCount * REC_INTERVAL / 1000);   // 整数算秒，避免浮点
    Serial.println(F(" 秒。按按键3 播放"));
    return;
  }

  recCount  = 0;                        // 第一次按：开始
  recLastMs = millis();
  recordMode = MODE_RECORD;
  isMoving   = false;
  Serial.println(F("==== 开始录制（用摇杆操控，再按按键2 结束）===="));
}

// 录制采样，每 REC_INTERVAL 采一个点
void recordTick()
{
  if(recordMode != MODE_RECORD) return;
  if(millis() - recLastMs < REC_INTERVAL) return;

  recLastMs += REC_INTERVAL;            // 累加而非赋值，保持长期周期准确

  if(recCount >= REC_MAX)               // 满了自动结束，避免越界
  {
    Serial.println(F("→ 缓存已满，自动结束录制"));
    toggleRecord();
    return;
  }

  writeRecord(recCount);
  recCount++;
}

// 按键3：播放
void startPlay()
{
  if(recCount < 2)
  {
    Serial.println(F("→ 没有可播放的动作，先按按键2 录制"));
    return;
  }

  playIndex  = 0;
  playLastMs = millis();
  recordMode = MODE_PLAY;
  isMoving   = false;

  Serial.print(F("==== 开始播放（共 "));
  Serial.print(recCount);
  Serial.println(F(" 点）===="));
}

// 播放推进
void playTick()
{
  if(recordMode != MODE_PLAY) return;
  if(millis() - playLastMs < REC_INTERVAL) return;
  playLastMs += REC_INTERVAL;

  playIndex++;

  if(playIndex >= recCount)             // 播完，精确对齐到最后一个点
  {
    int last[4];
    readRecord(recCount - 1, last);
    angX = last[0]; angY = last[1]; angZ = last[2]; angG = last[3];
    servoX.write(angX); servoY.write(angY); servoZ.write(angZ); servoG.write(angG);
    lastX = angX; lastY = angY; lastZ = angZ; lastG = angG;

    recordMode = MODE_MANUAL;
    Serial.println(F("==== 播放完成 ===="));
    return;
  }

  int want[4];
  readRecord(playIndex, want);

  /* 相邻采样点可能差很多度（录制时手快），直接跳过去会猛地一抖，
   * 所以按 STEP_DEG 限幅，大跨度自动拉成几个周期走完 */
  if(want[0] - angX >  STEP_DEG) want[0] = angX + STEP_DEG;
  if(want[0] - angX < -STEP_DEG) want[0] = angX - STEP_DEG;
  if(want[1] - angY >  STEP_DEG) want[1] = angY + STEP_DEG;
  if(want[1] - angY < -STEP_DEG) want[1] = angY - STEP_DEG;
  if(want[2] - angZ >  STEP_DEG) want[2] = angZ + STEP_DEG;
  if(want[2] - angZ < -STEP_DEG) want[2] = angZ - STEP_DEG;
  if(want[3] - angG >  STEP_DEG) want[3] = angG + STEP_DEG;
  if(want[3] - angG < -STEP_DEG) want[3] = angG - STEP_DEG;

  angX = constrainAngle(want[0], X_MIN, X_MAX);
  angY = constrainAngle(want[1], Y_MIN, Y_MAX);
  angZ = constrainAngle(want[2], Z_MIN, Z_MAX);
  angG = constrainAngle(want[3], G_OPEN, G_CLOSE);

  servoX.write(angX); lastX = angX;
  servoY.write(angY); lastY = angY;
  servoZ.write(angZ); lastZ = angZ;
  servoG.write(angG); lastG = angG;
}

/* ====================== 七、任务三：按键扫描 ====================== */

// 按键动作分发。k：0=循环 1=录制 2=播放 3=回中
void keyAction(int k)
{
  /* 按键4 回中无条件响应，因为它是软急停：
   * 搬运中、录制中、播放中都必须能立刻停下 */
  if(k == 3)
  {
    goHome();
    return;
  }

  // 录制中只认"结束录制"，避免录到一半切去搬运搞乱数据
  if(recordMode == MODE_RECORD)
  {
    if(k == 1) toggleRecord();
    return;
  }

  // 播放中忽略其余按键，避免和回放争抢姿态
  if(recordMode == MODE_PLAY) return;

  if(k == 0) cycleTask2();       // 按键1：循环执行 A→B→C
  if(k == 1) toggleRecord();     // 按键2：开始录制
  if(k == 2) startPlay();        // 按键3：播放
}

/* 四按键扫描（软件消抖）。
 * 读数变化时只记录时刻，连续稳定超过 25ms 才认作有效变化，
 * 并在"由高变低"（按下）时触发一次，按住不放不会重复触发。
 * 引脚为 INPUT_PULLUP，所以按下是低电平 */
void keyScan()
{
  /* ★ 上电保护期：启动后 500ms 内不响应按键。
   *
   * 为什么要加：打开串口监视器时，USB 转串口芯片会拉 DTR 让主控复位。
   * 复位瞬间各引脚电平不稳定，加上外部接线（按键、外设板）的瞬态，
   * D3（按键2＝录制）可能被瞬时拉低，被误判成"按下了录制键"。
   * 消抖逻辑无法区分"真实的按下"和"复位瞬间的低电平"，
   * 因为这两者都满足"稳定低电平超过 25ms"。
   *
   * 500ms 足够覆盖复位后的电平建立过程，而人手动按键不会这么快，
   * 所以不会影响正常操作。 */
  if(millis() < 500) return;

  static uint8_t raw[4]    = { HIGH, HIGH, HIGH, HIGH };
  static uint8_t stable[4] = { HIGH, HIGH, HIGH, HIGH };
  static unsigned long changeMs[4];

  for(int k = 0; k < 4; k++)
  {
    uint8_t now = digitalRead(keyPins[k]);

    if(now != raw[k])               // 还在抖动，只重新计时
    {
      raw[k] = now;
      changeMs[k] = millis();
      continue;
    }

    if(now != stable[k] && (millis() - changeMs[k]) > 25)
    {
      stable[k] = now;
      if(stable[k] == LOW) keyAction(k);
    }
  }
}

/* ====================== 八、串口指令解析 ======================
 * ★ 刻意不用 String：它会让静态 SRAM 多出上千字节
 *   （2KB 的 328P 吃不下），而且反复拼接会产生堆碎片。
 *   这里用固定长度 char 缓冲区逐字符解析。
 *   所有输出字符串都用 F() 包住，否则会占 RAM（见文件头第 3 条）。
 * ============================================================ */

const int CMD_BUF = 48;         // 最长指令约 21 字符，48 足够

char    cmdBuf[CMD_BUF];
uint8_t cmdLen;                 // 已收字符数
bool    cmdOverflow;            // 本行超长，整体丢弃
unsigned long cmdLastMs;        // 最后字符到达时刻，用于空闲超时

const int CMD_IDLE_MS = 200;    // 收到一半没动静就丢弃，防止两条指令粘连

void printHelp()
{
  Serial.println(F("O 开爪  S 关爪  H 提速  L 降速"));
  Serial.println(F("x90,y80,z70  同步设角度"));
  Serial.println(F("A / B / C    自动夹取搬运（任务二）"));
  Serial.println(F("R 录制起止   P 播放   M 回中   Q 状态   ? 帮助"));
  Serial.println(F("T40,80,90    平滑移动（标定用）"));
  Serial.println(F("按键1循环夹取  按键2录制  按键3播放  按键4回中"));
}

// 打印状态。标定点位时靠它读角度：摆到位 → 发 Q → 填进点位表
void printStatus()
{
  Serial.print(F("状态："));

  if(seqRunning)                     Serial.print(F("搬运中"));
  else if(recordMode == MODE_RECORD) Serial.print(F("录制中"));
  else if(recordMode == MODE_PLAY)   Serial.print(F("播放中"));
  else if(isMoving)                  Serial.print(F("移动中"));
  else                               Serial.print(F("待机"));

  Serial.print(F("  步长:")); Serial.print(speedStep);
  Serial.print(F("  下次按键1搬:"));
  Serial.print(key1Index == 0 ? F("A") : (key1Index == 1) ? F("B") : F("C"));
  Serial.print(F("  已录:")); Serial.print(recCount);
  Serial.println(F("点"));

  Serial.print(F("  X:")); Serial.print(angX);
  Serial.print(F(" Y:"));  Serial.print(angY);
  Serial.print(F(" Z:"));  Serial.print(angZ);
  Serial.print(F(" G:"));  Serial.println(angG);
}

/* 解析一个轴字段，返回轴号，-1 表示不是轴字母。
 * i 是引用：解析完把下标推到字段之后，供调用方循环调用 */
int parseAxis(const char *s, int &i, int &val)
{
  char c = s[i];
  int j = -1;
  if     (c == 'X') j = 0;
  else if(c == 'Y') j = 1;
  else if(c == 'Z') j = 2;
  else if(c == 'G') j = 3;
  if(j < 0) return -1;

  i++;
  if(s[i] == '=') i++;            // x90 和 x=90 都支持
  if(s[i] < '0' || s[i] > '9') return -1;

  int v = 0;
  while(s[i] >= '0' && s[i] <= '9')
  {
    v = v * 10 + (s[i] - '0');
    i++;
  }
  while(s[i] == ',' || s[i] == ' ') i++;

  val = v;
  return j;
}

// 执行一条已转大写、已去行尾的指令
void executeCommand(char *cmd)
{
  /* 剥离自制板板号前缀（B + 数字 + 合法指令）。
   * ★ 必须在下面分派之前判断，否则 "B1C" 会先被当成"搬运物体 B"。
   *   且只有剩下的字符属于合法指令首字母才认作板号，
   *   否则 "BC" 会被误拆成"板号 B + 夹物体 C" */
  if(cmd[0] == 'B' && cmd[1] >= '0' && cmd[1] <= '9')
  {
    int i = 1;
    while(cmd[i] >= '0' && cmd[i] <= '9') i++;
    char head = cmd[i];
    if(head == 'O' || head == 'S' || head == 'H' || head == 'L' ||
       head == 'A' || head == 'B' || head == 'C' || head == 'R' ||
       head == 'P' || head == 'M' || head == 'Q' || head == '?' ||
       head == 'X' || head == 'Y' || head == 'Z' || head == 'G' ||
       head == 'T')
    {
      cmd += i;
    }
  }
  if(cmd[0] == '\0') return;

  /* 以下三条任何时候都响应，必须放在"忙碌判断"之前：
   *   M 是软急停，R 要能随时结束录制，Q/? 只是查询不改变状态 */
  if(cmd[0] == 'M' && cmd[1] == '\0') { goHome();      return; }
  if(cmd[0] == 'Q' && cmd[1] == '\0') { printStatus(); return; }
  if(cmd[0] == '?' && cmd[1] == '\0') { printHelp();   return; }
  if(cmd[0] == 'R' && cmd[1] == '\0') { toggleRecord();return; }
  if(cmd[0] == 'P' && cmd[1] == '\0') { startPlay();   return; }

  // 任务一单字符指令
  if(cmd[0] == 'O' && cmd[1] == '\0') { angG = G_OPEN;  servoG.write(angG); lastG = angG; Serial.println(F("→爪子张开")); return; }
  if(cmd[0] == 'S' && cmd[1] == '\0') { angG = G_CLOSE; servoG.write(angG); lastG = angG; Serial.println(F("→爪子闭合")); return; }

  if(cmd[0] == 'H' && cmd[1] == '\0')
  {
    speedStep++; if(speedStep > 8) speedStep = 8;
    Serial.print(F("→提速，步长：")); Serial.println(speedStep);
    return;
  }
  if(cmd[0] == 'L' && cmd[1] == '\0')
  {
    speedStep--; if(speedStep < 1) speedStep = 1;
    Serial.print(F("→降速，步长：")); Serial.println(speedStep);
    return;
  }

  // 剩余的指令都会改变姿态，忙碌时一律拒绝
  if(seqRunning || isMoving || recordMode != MODE_MANUAL)
  {
    Serial.println(F("→ 忙碌中（按 M 或按键4 可立即中止）"));
    return;
  }

  // 任务二
  if(cmd[0] == 'A' && cmd[1] == '\0') { runTask2(0); return; }
  if(cmd[0] == 'B' && cmd[1] == '\0') { runTask2(1); return; }
  if(cmd[0] == 'C' && cmd[1] == '\0') { runTask2(2); return; }

  /* 任务一多舵机协同 x10,y30,z20：只设出现的轴，其余保持不动。
   * 判定时要求第 2 字符是数字或等号，否则 "ZZZ" 会被误当轴指令 */
  bool isAxis = (cmd[0]=='X' || cmd[0]=='Y' || cmd[0]=='Z' || cmd[0]=='G') &&
                ((cmd[1] >= '0' && cmd[1] <= '9') || cmd[1] == '=');
  if(isAxis)
  {
    int v[4];
    bool seen[4] = { false, false, false, false };
    v[0] = v[1] = v[2] = v[3] = 0;

    int i = 0;
    while(cmd[i])
    {
      int val = 0;
      int j = parseAxis(cmd, i, val);
      if(j < 0) break;           // 不合法就停（此时下标没推进，不会死循环）
      v[j] = val;
      seen[j] = true;
    }

    if(!seen[0] && !seen[1] && !seen[2] && !seen[3])
    {
      Serial.println(F("→ 格式错误，示例 x90,y80,z70"));
      return;
    }

    // 立即写值（考核要求输出对应 PWM）。被限幅的轴回显时加 "!"
    Serial.print(F("→"));
    if(seen[0]){ int r = constrainAngle(v[0], X_MIN, X_MAX); angX = r; lastX = r; servoX.write(r); Serial.print(F(" X=")); Serial.print(r); if(r != v[0]) Serial.print('!'); }
    if(seen[1]){ int r = constrainAngle(v[1], Y_MIN, Y_MAX); angY = r; lastY = r; servoY.write(r); Serial.print(F(" Y=")); Serial.print(r); if(r != v[1]) Serial.print('!'); }
    if(seen[2]){ int r = constrainAngle(v[2], Z_MIN, Z_MAX); angZ = r; lastZ = r; servoZ.write(r); Serial.print(F(" Z=")); Serial.print(r); if(r != v[2]) Serial.print('!'); }
    if(seen[3]){ int r = constrainAngle(v[3], G_OPEN, G_CLOSE); angG = r; lastG = r; servoG.write(r); Serial.print(F(" G=")); Serial.print(r); if(r != v[3]) Serial.print('!'); }
    Serial.println();
    return;
  }

  // 标定用 T40,80,90：平滑移动，最后一个参数 -1 表示夹爪不动
  if(cmd[0] == 'T')
  {
    int i = 1;
    int idx[2] = { -1, -1 };
    int n = 0;
    while(cmd[i] && n < 2)
    {
      if(cmd[i] == ',') idx[n++] = i;
      i++;
    }
    if(n < 2)
    {
      Serial.println(F("格式错误，示例 T40,80,90"));
      return;
    }

    int tx = 0, ty = 0, tz = 0;
    for(int k = 1;       k < idx[0]; k++) tx = tx * 10 + (cmd[k] - '0');
    for(int k = idx[0]+1; k < idx[1]; k++) ty = ty * 10 + (cmd[k] - '0');
    for(int k = idx[1]+1; cmd[k];     k++) tz = tz * 10 + (cmd[k] - '0');

    moveToStart(tx, ty, tz, -1);
    Serial.println(F("→ 平滑移动中"));
    return;
  }

  Serial.println(F("→ 未知指令，发 ? 看帮助"));
}

/* 逐字符收串口，遇 \n 执行整条指令。
 * 不用 readStringUntil：它会阻塞最多 1 秒，且超时后残留字符
 * 会和下一条指令粘成乱码，还会引入 String 的堆碎片 */
void serialPoll()
{
  while(Serial.available())
  {
    char c = (char)Serial.read();
    cmdLastMs = millis();

    if(c == '\r') continue;          // 兼容 CRLF 行尾

    if(c == '\n')
    {
      cmdBuf[cmdLen] = '\0';

      if(cmdOverflow) Serial.println(F("→ 指令过长，已丢弃"));
      else if(cmdLen > 0)
      {
        for(uint8_t i = 0; i < cmdLen; i++)      // 转大写，只处理 ASCII
        {
          if(cmdBuf[i] >= 'a' && cmdBuf[i] <= 'z') cmdBuf[i] -= 32;
        }
        Serial.print(F("收到：")); Serial.println(cmdBuf);
        executeCommand(cmdBuf);
      }

      cmdLen = 0;
      cmdOverflow = false;
      continue;
    }

    /* ★ 字符白名单：只接受指令真正会用到的字符，其余一律丢弃。
     *
     * 不加这一层的话，RX 线上任何一个杂散字节都会被当成指令执行。
     * 曾经出现过"打开串口监视器就自动开始录制"的现象，原因就是
     * 一个噪声字节恰好是 'R'（0x52），而 'R' 正是"录制起止"指令。
     *
     * 白名单覆盖：大写字母（指令）、小写字母（稍后转大写）、数字（角度）、
     * 逗号与空格（分隔符）、等号（x=90 写法）。
     * 其余字节（噪声、串口连接瞬间的杂散数据、外部板吐的乱码）直接扔掉，
     * 既不进缓冲也不影响已收内容。 */
    bool valid = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                 (c >= '0' && c <= '9') ||
                 c == ',' || c == ' ' || c == '=';
    if(!valid) continue;

    if(cmdLen < CMD_BUF - 1) cmdBuf[cmdLen++] = c;
    else                     cmdOverflow = true;
  }

  if(cmdLen > 0 && (millis() - cmdLastMs) > CMD_IDLE_MS)
  {
    cmdLen = 0;
    cmdOverflow = false;
  }
}

/* ====================== 九、setup / loop ====================== */

void setup()
{
  /* 全局状态在这里赋初值，让它们落在 .bss 段省 Flash */
  angX = HOME_X;  angY = HOME_Y;  angZ = HOME_Z;  angG = HOME_G;
  lastX = angX;   lastY = angY;   lastZ = angZ;   lastG = angG;

  speedStep  = 2;
  recordMode = MODE_MANUAL;
  recCount   = 0;
  playIndex  = 0;

  seqRunning = false;
  seqObj     = 0;
  seqIndex   = 0;
  key1Index  = 0;

  isMoving   = false;
  arriveAt   = 0;
  goalX = goalY = goalZ = goalG = 0;

  cmdLen = 0;
  cmdOverflow = false;
  cmdLastMs = 0;

  Serial.begin(9600);

  // 按键用内部上拉，按下为低电平，不需要外接电阻
  for(int k = 0; k < 4; k++)
  {
    pinMode(keyPins[k], INPUT_PULLUP);
  }

  // 上电即摆到 HOME 位姿，避免通电瞬间机械臂甩向某个方向
  servoX.attach(9);
  servoY.attach(8);
  servoZ.attach(7);
  servoG.attach(6);

  servoX.write(angX);
  servoY.write(angY);
  servoZ.write(angZ);
  servoG.write(angG);

  delay(500);                    // 等舵机到位姿再开始收指令

  Serial.println(F("MeArm 任务一 + 任务二 + 任务三 就绪"));
  printHelp();
}

void loop()
{
  /* 顺序有讲究：
   * 1) 先收串口 —— 保证 M 急停最及时
   * 2) 再扫按键 —— 按键4 是无条件急停，要在动作推进之前
   * 3) 按模式推进 —— 搬运 / 录制 / 播放 / 单次移动，四者互斥
   * 4) 最后读摇杆 —— 只在完全待机时才响应，避免和自动动作打架
   * 全程没有为了等待而写很长的 delay */
  serialPoll();
  keyScan();

  if(seqRunning)
  {
    seqUpdate();
  }
  else if(recordMode == MODE_RECORD)
  {
    joystickControl();           // 录制中仍由摇杆操控
    recordTick();                // 同时按周期采样
  }
  else if(recordMode == MODE_PLAY)
  {
    playTick();
  }
  else
  {
    moveUpdate();                // 单次平滑移动（T 指令、回中）

    if(!isMoving) joystickControl();
  }

  delay(20);                     // 控制周期，影响所有速度
}
