#include <Servo.h>

Servo servoX;   //X：底座旋转
Servo servoY;   //Y：后臂（L1）
Servo servoZ;   //Z：前臂（L2），角度越大夹爪越低
Servo servoG;   //G：夹爪（夹住铅笔，绘图时基本不动）

/* =====================================================================
 *  一、引脚与任务三的按键
 * ===================================================================== */

const int joy1X = A0;    // 摇杆1 水平 → 控制 X
const int joy1Y = A1;    // 摇杆1 垂直 → 控制 Y
const int joy2X = A2;    // 摇杆2 水平 → 控制 Z
const int joy2Y = A3;    // 摇杆2 垂直 → 控制 G

const int keyPin1 = 2;   // 击停/继续
const int keyPin2 = 3;   // 取消
const int keyPin3 = 4;   // 记录示教点
const int keyPin4 = 5;   // 回中
const int keyPins[4] = { keyPin1, keyPin2, keyPin3, keyPin4 };

/* =====================================================================
 *  二、舵机限位与连杆参数
 * ===================================================================== */

const int X_MIN = 0,  X_MAX = 180;
const int Y_MIN = 45, Y_MAX = 125;
const int Z_MIN = 60, Z_MAX = 130;
const int G_OPEN = 20, G_CLOSE = 100;
const int G_PEN  = 60;        // 夹住铅笔时的夹爪角度

const int HOME_X = 90, HOME_Y = 90, HOME_Z = 90;

/* ---- 连杆参数（任务四文档中的 L0 / α / L1 / L2）----
 * ★ 必须按实物标定：SolidWorks 里量铰接孔中心距，或卡尺实测。
 *   这里给的是估计值，直接影响绘图精度。 */
const float L0    = 40.0;      // 转轴到第一根杆最低点（mm）
const float ALPHA = 20.0;      // L0 与水平面夹角（度）
const float L1    = 80.0;      // 后臂长度（mm）
const float L2    = 70.0;      // 前臂长度（mm）

/* ---- 几何角 ←→ 舵机角 的偏置 ----
 * 任务四的 θ1、θ2 是"与水平面的几何夹角"，舵机读数是另一套刻度。
 *   θ1 = 舵机Y + OFF_Y
 *   θ2 = 舵机Z + OFF_Z
 * 标定方法：拆下连杆让后臂水平（几何角 0°），读此时舵机值即 -OFF。
 * ★ 这两个值填错，机械臂会朝反方向走，务必先标定。 */
const float OFF_Y = 0.0;
const float OFF_Z = 0.0;

/* =====================================================================
 *  三、绘图参数
 * ===================================================================== */

/* 纸面高度（工程 z，mm）。z 越小越低。
 * 由工作空间分析确定：本机在 z = 30~50 mm 处可用水平宽度最大（约 185 mm）。
 * ★ 标定：把笔尖轻触纸面，发 Q 读出当前 z，填进来。 */
const float PEN_DOWN_Z = 40.0;

/* 抬笔时 z 增加多少（mm）。越大抬得越高、越安全，但耗时更长 */
const float PEN_UP_DZ  = 12.0;

/* 笔尖相对末端夹爪的固定偏移（mm）。铅笔装偏了就在这里补 */
const float TIP_DX = 0.0;
const float TIP_DY = 0.0;
const float TIP_DZ = 0.0;

/* 绘图区域（相对底座转轴的半径范围，mm）。
 * 由工作空间分析得出：z≈40mm 时可用半径 0~185，取保守值。 */
const float DRAW_R_MIN = 60.0;
const float DRAW_R_MAX = 170.0;

/* 图形默认尺寸（弧长 / 边长，mm） */
const float SHAPE_SIZE = 70.0;

/* 直线插补的采样步长（mm）。越小越平滑，但动作点更多、更慢 */
const float SEG_STEP_MM = 2.0;

/* 平滑曲线的每段采样点数 */
const uint8_t CURVE_SAMPLES = 10;

/* 每个绘图点的保持时间（ms）。给舵机留出到位时间 */
const uint16_t DRAW_POINT_HOLD = 25;

/* 运动每个控制周期的最大角度增量（度），限制速度防止抖动 */
const float STEP_DEG = 2.0;

/* =====================================================================
 *  四、全局状态
 * ===================================================================== */

int angX, angY, angZ, angG;             // 当前关节角（任务一~三沿用）
int lastX, lastY, lastZ, lastG;
int speedStep;

/* 运动目标与限幅 */
float tgtX, tgtY, tgtZ, tgtG;           // 内部用 float 算，下发前取整
bool  isMoving;
unsigned long arriveAt;

/* 工作模式 */
enum Mode : uint8_t { MODE_IDLE, MODE_MANUAL, MODE_TEACH, MODE_DRAW };
Mode mode;

/* 绘图过程控制 */
bool drawPaused   = false;              // 是否处于暂停
bool drawCancel   = false;              // 是否请求取消
uint8_t drawCursor = 0;                 // 当前执行到第几个点
uint16_t drawCount = 0;                 // 本次轨迹总点数
unsigned long drawHoldUntil = 0;        // 当前点的保持截止时刻

/* 轨迹缓冲：每个点存 4 字节（θ0 / θ1 / θ2 各 1 字节 + 抬笔标志）。
 *
 * 为什么能压到 1 字节存一个角？
 *   逆解算出来的几何角经过舵机限位后本来就是整数度，
 *   而舵机本身的分辨率也就 1 度左右，所以用 int8_t 装"相对基准角的偏差"
 *   不损失实际精度：
 *     θ0 约定装在 (-90, 90] 区间（横竖两个方向取一支即可）
 *     θ1、θ2 是几何角，范围 -100~+100 度
 *
 * 240 点 × 4 字节 = 960 字节。
 * ★ 这个数组是本程序最大的一块内存，改大前先看编译输出的 Data 百分比：
 *   超过 100% 会爆栈，表现为随机重启。 */
const uint16_t TRAJ_MAX = 240;

struct TrajPoint
{
  int8_t th0;      // 几何角 θ0 − 90（度）
  int8_t th1;      // 几何角 θ1（度）
  int8_t th2;      // 几何角 θ2（度）
  uint8_t flags;   // bit0：该点是否抬笔经过
};
TrajPoint traj[TRAJ_MAX];

/* 示教点（最多 8 个，考核要 5 个） */
const uint8_t TEACH_MAX = 8;
float teachX[TEACH_MAX], teachY[TEACH_MAX], teachZ[TEACH_MAX];
uint8_t teachCount;

/* 串口缓冲 */
const int CMD_BUF = 48;
char    cmdBuf[CMD_BUF];
uint8_t cmdLen;
bool    cmdOverflow;
unsigned long cmdLastMs;
const int CMD_IDLE_MS = 200;

/* 当前使用哪个肘部支：-1 肘下，+1 肘上。
 * 整条轨迹必须固定用同一支，否则中途翻支会让机械臂突跳。
 * 标定后若 -1 支的画幅不够，改成 +1。 */
const int IK_BRANCH = -1;

/* =====================================================================
 *  五、基础工具
 * ===================================================================== */

int constrainAngle(int val, int minA, int maxA)
{
  if(val < minA) return minA;
  if(val > maxA) return maxA;
  return val;
}

float constrainF(float v, float lo, float hi)
{
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

int16_t clampAngle(int16_t v, int lo, int hi)
{
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

void flushServos()
{
  if(angX != lastX){ servoX.write(angX); lastX = angX; }
  if(angY != lastY){ servoY.write(angY); lastY = angY; }
  if(angZ != lastZ){ servoZ.write(angZ); lastZ = angZ; }
  if(angG != lastG){ servoG.write(angG); lastG = angG; }
}

/* =====================================================================
 *  六、运动学（实现任务四文档的公式）
 *
 *  正解：  R = L0cosα + L1cosθ1 + L2cosθ2
 *          x = R cosθ0 ,  y = R sinθ0 ,  z = L0sinα + L1sinθ1 + L2sinθ2
 *  逆解：  θ0 = atan2(y0, x0)
 *          A  = z0 - L0 sinα ,  B = √(x0²+y0²) - L0 cosα
 *          γ  = ± arccos[(A²+B²-L1²-L2²)/(2 L1 L2)]
 *          θ1 = atan2(A,B) - atan2(L2 sinγ, L1 + L2 cosγ)
 *          θ2 = θ1 + γ
 * ===================================================================== */

/* 几何角 → 舵机角 */
void geomToServo(float th0, float th1, float th2, int &sx, int &sy, int &sz)
{
  sx = constrainAngle((int)lround(th0), X_MIN, X_MAX);
  sy = constrainAngle((int)lround(th1 - OFF_Y), Y_MIN, Y_MAX);
  sz = constrainAngle((int)lround(th2 - OFF_Z), Z_MIN, Z_MAX);
}

/* 舵机角 → 几何角 */
void servoToGeom(int sx, int sy, int sz, float &th0, float &th1, float &th2)
{
  th0 = (float)sx;
  th1 = (float)sy + OFF_Y;
  th2 = (float)sz + OFF_Z;
}

/* 逆解。branch：+1 / -1 选肘部支。
 * 返回 false 表示不可达或超出关节限位。 */
bool ik(float x, float y, float z, int branch, int &sx, int &sy, int &sz,
        float &th0o, float &th1o, float &th2o)
{
  const float DEG = 57.29577951f;

  float th0 = atan2f(y, x);
  float a   = ALPHA / DEG;

  float A = z - L0 * sinf(a);
  float B = sqrtf(x * x + y * y) - L0 * cosf(a);

  float d2 = A * A + B * B;
  float dmax = (L1 + L2) * (L1 + L2);
  float dmin = (L1 - L2) * (L1 - L2);
  if(d2 > dmax || d2 < dmin) return false;      // 目标点不可达

  float cg = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  cg = constrainF(cg, -1.0f, 1.0f);

  float g  = (branch >= 0) ? acosf(cg) : -acosf(cg);
  float p  = L1 + L2 * cosf(g);
  float q  = L2 * sinf(g);
  float th1 = atan2f(A, B) - atan2f(q, p);
  float th2 = th1 + g;

  /* 几何角换算成舵机角，并检查是否在限位内 */
  float f0 = th0 * DEG, f1 = th1 * DEG, f2 = th2 * DEG;
  int c0 = constrainAngle((int)lround(f0), X_MIN, X_MAX);
  int c1 = constrainAngle((int)lround(f1 - OFF_Y), Y_MIN, Y_MAX);
  int c2 = constrainAngle((int)lround(f2 - OFF_Z), Z_MIN, Z_MAX);

  /* 限位把角度夹变了超过 3 度，就认为该点不可用，避免画出错误轨迹 */
  if(abs(c0 - (int)lround(f0)) > 3) return false;
  if(abs(c1 - (int)lround(f1 - OFF_Y)) > 3) return false;
  if(abs(c2 - (int)lround(f2 - OFF_Z)) > 3) return false;

  sx = c0; sy = c1; sz = c2;
  th0o = f0; th1o = f1; th2o = f2;
  return true;
}

/* 正解：舵机角 → (x, y, z)。
 * 用于把摇杆摆出的姿态读成坐标，方便示教时确认位置。 */
void fk(int sx, int sy, int sz, float &x, float &y, float &z)
{
  const float DEG = 57.29577951f;
  float th0, th1, th2;
  servoToGeom(sx, sy, sz, th0, th1, th2);
  float a = ALPHA / DEG;
  float r1 = th1 / DEG, r2 = th2 / DEG;
  float R = L0 * cosf(a) + L1 * cosf(r1) + L2 * cosf(r2);
  float t0 = th0 / DEG;
  x = R * cosf(t0);
  y = R * sinf(t0);
  z = L0 * sinf(a) + L1 * sinf(r1) + L2 * sinf(r2);
}

/* =====================================================================
 *  七、非阻塞移动（只按关节角走，不做笛卡尔插值）
 * ===================================================================== */

void moveToStart(float x, float y, float z, float g)
{
  tgtX = (float)constrainAngle((int)lround(x), X_MIN, X_MAX);
  tgtY = (float)constrainAngle((int)lround(y), Y_MIN, Y_MAX);
  tgtZ = (float)constrainAngle((int)lround(z), Z_MIN, Z_MAX);
  if(g >= 0) tgtG = (float)constrainAngle((int)lround(g), G_OPEN, G_CLOSE);
  else       tgtG = (float)angG;

  isMoving = true;
  arriveAt = 0;
}

bool moveUpdate(int holdMs)
{
  if(!isMoving) return true;

  float dX = tgtX - angX, dY = tgtY - angY, dZ = tgtZ - angZ, dG = tgtG - angG;

  if(dX >  STEP_DEG) angX += (int)STEP_DEG; else if(dX < -STEP_DEG) angX -= (int)STEP_DEG; else angX = (int)tgtX;
  if(dY >  STEP_DEG) angY += (int)STEP_DEG; else if(dY < -STEP_DEG) angY -= (int)STEP_DEG; else angY = (int)tgtY;
  if(dZ >  STEP_DEG) angZ += (int)STEP_DEG; else if(dZ < -STEP_DEG) angZ -= (int)STEP_DEG; else angZ = (int)tgtZ;
  if(dG >  STEP_DEG) angG += (int)STEP_DEG; else if(dG < -STEP_DEG) angG -= (int)STEP_DEG; else angG = (int)tgtG;

  flushServos();

  if(angX == (int)tgtX && angY == (int)tgtY && angZ == (int)tgtZ && angG == (int)tgtG)
  {
    if(arriveAt == 0) arriveAt = millis();
    if(millis() - arriveAt >= (unsigned long)holdMs)
    {
      isMoving = false;
      return true;
    }
  }
  return false;
}

/* =====================================================================
 *  八、轨迹生成
 * ===================================================================== */

void trajReset() { drawCount = 0; }

/* 往轨迹里追加一个笛卡尔点。返回 false 表示该点不可达、已跳过 */
bool trajAddPoint(float x, float y, float z, bool penUp)
{
  if(drawCount >= TRAJ_MAX) return false;

  int sx, sy, sz;
  float a, b, c;
  if(!ik(x, y, z, IK_BRANCH, sx, sy, sz, a, b, c)) return false;

  /* 存几何角（整数度）。θ0 需落在 (-90, 90]：
   * 底座是对称的，把角度折算到这一区间后绘图效果完全相同。 */
  float t0 = a;
  while(t0 >  90.0f) t0 -= 180.0f;
  while(t0 <= -90.0f) t0 += 180.0f;

  traj[drawCount].th0   = (int8_t)lround(t0 - 90.0f);
  traj[drawCount].th1   = (int8_t)lround(b);
  traj[drawCount].th2   = (int8_t)lround(c);
  traj[drawCount].flags = penUp ? 1 : 0;
  drawCount++;
  return true;
}

/* 在两点之间做直线插补 */
void trajLine(float x0, float y0, float x1, float y1, float z, bool penUp)
{
  float dx = x1 - x0, dy = y1 - y0;
  float len = sqrtf(dx * dx + dy * dy);
  uint16_t n = (uint16_t)(len / SEG_STEP_MM);
  if(n < 1) n = 1;
  for(uint16_t i = 0; i <= n; i++)
  {
    float t = (float)i / (float)n;
    trajAddPoint(x0 + dx * t, y0 + dy * t, z, penUp);
  }
}

/* Catmull-Rom 平滑曲线：经过所有控制点，C1 连续。
 * 与折线的区别就是"点与点之间不是直线，而是样条"。 */
void trajCatmullRom(float *px, float *py, uint8_t n, float z, bool penUp)
{
  for(uint8_t i = 0; i < n - 1; i++)
  {
    /* 端点用相邻点复制，避免越界 */
    float x0 = (i == 0)     ? px[0]     : px[i - 1];
    float y0 = (i == 0)     ? py[0]     : py[i - 1];
    float x1 = px[i],       y1 = py[i];
    float x2 = px[i + 1],   y2 = py[i + 1];
    float x3 = (i + 2 < n)  ? px[i + 2] : px[n - 1];
    float y3 = (i + 2 < n)  ? py[i + 2] : py[n - 1];

    for(uint8_t s = 0; s < CURVE_SAMPLES; s++)
    {
      float t  = (float)s / (float)CURVE_SAMPLES;
      float t2 = t * t, t3 = t2 * t;

      float x = 0.5f * ((2.0f * x1) + (-x0 + x2) * t +
                        (2.0f * x0 - 5.0f * x1 + 4.0f * x2 - x3) * t2 +
                        (-x0 + 3.0f * x1 - 3.0f * x2 + x3) * t3);
      float y = 0.5f * ((2.0f * y1) + (-y0 + y2) * t +
                        (2.0f * y0 - 5.0f * y1 + 4.0f * y2 - y3) * t2 +
                        (-y0 + 3.0f * y1 - 3.0f * y2 + y3) * t3);

      trajAddPoint(x, y, z, penUp);
    }
  }
  trajAddPoint(px[n - 1], py[n - 1], z, penUp);   // 补上最后一个控制点
}

/* 极坐标（半径、角度）转笛卡尔，方便在绘图区里排版图形 */
void polarToXY(float r, float deg, float &x, float &y)
{
  float t = deg / 57.29577951f;
  x = r * cosf(t);
  y = r * sinf(t);
}

/* =====================================================================
 *  九、子任务 1~2：预设图形
 * ===================================================================== */

/* 子任务1：直线 */
void buildLine()
{
  trajReset();
  float r0 = DRAW_R_MIN, r1 = DRAW_R_MAX;
  float x0, y0, x1, y1;

  polarToXY(r0, -30, x0, y0);
  polarToXY(r1, -30, x1, y1);

  /* 先抬笔移到起点，再落笔画到终点，最后抬笔 */
  trajAddPoint(x0, y0, PEN_DOWN_Z + PEN_UP_DZ, true);
  trajLine(x0, y0, x1, y1, PEN_DOWN_Z, false);
  trajAddPoint(x1, y1, PEN_DOWN_Z + PEN_UP_DZ, true);

  Serial.print(F("→ 直线轨迹点数 "));
  Serial.println(drawCount);
}

/* 子任务2：基础图形。kind：0=N 1=Z 2=V 3=三角形 */
void buildShape(uint8_t kind)
{
  trajReset();

  float cx, cy;
  polarToXY((DRAW_R_MIN + DRAW_R_MAX) * 0.5f, 0, cx, cy);   // 图形中心
  float S = SHAPE_SIZE;

  float xs[4], ys[4];
  uint8_t n = 0;

  switch(kind)
  {
    case 0:   /* 字母 N：左竖 + 斜线 + 右竖 */
    {
      float l = cx - S * 0.35f, r = cx + S * 0.35f;
      float b = cy - S * 0.5f,  t = cy + S * 0.5f;
      /* 抬笔到左下 → 画左竖 → 抬到左上 → 画斜线到底 → 抬到右下 → 画右竖 */
      float x0, y0;
      polarToXY(sqrtf(l*l + b*b), atan2f(b, l) * 57.29577951f, x0, y0);
      trajAddPoint(l, b, PEN_DOWN_Z + PEN_UP_DZ, true);
      trajLine(l, b, l, t, PEN_DOWN_Z, false);
      trajAddPoint(l, t, PEN_DOWN_Z, false);
      trajLine(l, t, r, b, PEN_DOWN_Z, false);
      trajAddPoint(r, b, PEN_DOWN_Z, false);
      trajLine(r, b, r, t, PEN_DOWN_Z, false);
      trajAddPoint(r, t, PEN_DOWN_Z + PEN_UP_DZ, true);
      break;
    }
    case 1:   /* 字母 Z：上横 + 斜线 + 下横 */
    {
      float l = cx - S * 0.35f, r = cx + S * 0.35f;
      float b = cy - S * 0.5f,  t = cy + S * 0.5f;
      trajAddPoint(l, t, PEN_DOWN_Z + PEN_UP_DZ, true);
      trajLine(l, t, r, t, PEN_DOWN_Z, false);
      trajAddPoint(r, t, PEN_DOWN_Z, false);
      trajLine(r, t, l, b, PEN_DOWN_Z, false);
      trajAddPoint(l, b, PEN_DOWN_Z, false);
      trajLine(l, b, r, b, PEN_DOWN_Z, false);
      trajAddPoint(r, b, PEN_DOWN_Z + PEN_UP_DZ, true);
      break;
    }
    case 2:   /* 字母 V：两笔斜线 */
    {
      float l = cx - S * 0.35f, r = cx + S * 0.35f;
      float b = cy - S * 0.5f,  t = cy + S * 0.5f;
      trajAddPoint(l, t, PEN_DOWN_Z + PEN_UP_DZ, true);
      trajLine(l, t, cx, b, PEN_DOWN_Z, false);
      trajAddPoint(cx, b, PEN_DOWN_Z, false);
      trajLine(cx, b, r, t, PEN_DOWN_Z, false);
      trajAddPoint(r, t, PEN_DOWN_Z + PEN_UP_DZ, true);
      break;
    }
    default:  /* 三角形：等边三角形，闭合 */
    {
      float R = S * 0.577f;                       // 外接圆半径
      for(uint8_t i = 0; i < 4; i++)
      {
        float deg = -90.0f + 120.0f * i;          // 从顶点开始
        float x, y;
        polarToXY(1.0f, deg, x, y);
        xs[n] = cx + R * x;
        ys[n] = cy + R * y;
        n++;
      }
      trajAddPoint(xs[0], ys[0], PEN_DOWN_Z + PEN_UP_DZ, true);
      for(uint8_t i = 0; i < 3; i++)
        trajLine(xs[i], ys[i], xs[i + 1], ys[i + 1], PEN_DOWN_Z, false);
      trajAddPoint(xs[3], ys[3], PEN_DOWN_Z + PEN_UP_DZ, true);
      break;
    }
  }

  Serial.print(F("→ 图形轨迹点数 "));
  Serial.println(drawCount);
}

/* 子任务5：五点示教 → 折线 */
void buildPolyline()
{
  trajReset();
  if(teachCount < 2)
  {
    Serial.println(F("→ 示教点不足，先用摇杆摆位并按 D4 / 发 G 记录"));
    return;
  }
  trajAddPoint(teachX[0], teachY[0], PEN_DOWN_Z + PEN_UP_DZ, true);
  for(uint8_t i = 0; i < teachCount; i++)
  {
    trajAddPoint(teachX[i], teachY[i], PEN_DOWN_Z, false);
    if(i + 1 < teachCount)
      trajLine(teachX[i], teachY[i], teachX[i + 1], teachY[i + 1], PEN_DOWN_Z, false);
  }
  trajAddPoint(teachX[teachCount - 1], teachY[teachCount - 1],
               PEN_DOWN_Z + PEN_UP_DZ, true);
  Serial.print(F("→ 折线轨迹点数 "));
  Serial.println(drawCount);
}

/* 子任务6：五点示教 → 平滑曲线 */
void buildCurve()
{
  trajReset();
  if(teachCount < 3)
  {
    Serial.println(F("→ 曲线至少需要 3 个示教点"));
    return;
  }
  trajAddPoint(teachX[0], teachY[0], PEN_DOWN_Z + PEN_UP_DZ, true);
  trajCatmullRom(teachX, teachY, teachCount, PEN_DOWN_Z, false);
  trajAddPoint(teachX[teachCount - 1], teachY[teachCount - 1],
               PEN_DOWN_Z + PEN_UP_DZ, true);
  Serial.print(F("→ 曲线轨迹点数 "));
  Serial.println(drawCount);
}

/* 前向声明：serialPoll 会调用它，而它定义在后面 */
void executeCommand();

/* =====================================================================
 *  十、轨迹执行（含暂停 / 继续 / 取消）
 * ===================================================================== */

void startDraw()
{
  if(drawCount == 0)
  {
    Serial.println(F("→ 轨迹为空"));
    mode = MODE_IDLE;
    return;
  }
  drawCursor = 0;
  drawPaused = false;
  drawCancel = false;
  mode = MODE_DRAW;
  Serial.print(F("==== 开始绘制，共 "));
  Serial.print(drawCount);
  Serial.println(F(" 点 ===="));
}

void stopDraw(const __FlashStringHelper *why)
{
  mode = MODE_IDLE;
  drawPaused = false;
  isMoving = false;
  Serial.print(F("==== "));
  Serial.print(why);
  Serial.println(F(" ===="));
}

void drawTick()
{
  /* 取消：立刻停止，并把笔抬起来离开纸面 */
  if(drawCancel)
  {
    drawCancel = false;
    stopDraw(F("已取消绘制"));
    moveToStart(angX, angY, PEN_DOWN_Z + PEN_UP_DZ, angG);   // 抬笔
    return;
  }

  /* 暂停：保持当前状态不动。注意不改变 drawCursor，
   * 所以继续时会从暂停的那个点接着画 */
  if(drawPaused)
  {
    return;
  }

  /* 当前点还没走完，继续走 */
  if(isMoving)
  {
    moveUpdate(DRAW_POINT_HOLD);
    return;
  }

  /* 当前点已完成，取下一个点 */
  if(drawCursor >= drawCount)
  {
    stopDraw(F("绘制完成"));
    return;
  }

  int sx, sy, sz;
  float a = (float)traj[drawCursor].th0 + 90.0f;    // 还原 θ0
  float b = (float)traj[drawCursor].th1;
  float c = (float)traj[drawCursor].th2;
  geomToServo(a, b, c, sx, sy, sz);

  moveToStart(sx, sy, sz, G_PEN);
  drawCursor++;
}

/* =====================================================================
 *  十一、摇杆手动 + 示教记录
 * ===================================================================== */

void joystickControl()
{
  int j1x = analogRead(joy1X), j1y = analogRead(joy1Y);
  int j2x = analogRead(joy2X), j2Y = analogRead(joy2Y);
  const int deadZone = 40;

  if(abs(j1x - 512) > deadZone) angX += (j1x > 512) ? speedStep : -speedStep;
  angX = constrainAngle(angX, X_MIN, X_MAX);

  if(abs(j1y - 512) > deadZone) angY += (j1y > 512) ? speedStep : -speedStep;
  angY = constrainAngle(angY, Y_MIN, Y_MAX);

  if(abs(j2x - 512) > deadZone) angZ += (j2x > 512) ? speedStep : -speedStep;
  angZ = constrainAngle(angZ, Z_MIN, Z_MAX);

  if(abs(j2Y - 512) > deadZone) angG += (j2Y > 512) ? speedStep : -speedStep;
  angG = constrainAngle(angG, G_OPEN, G_CLOSE);

  isMoving = false;              // 手动优先，放弃自动移动目标
  tgtX = angX; tgtY = angY; tgtZ = angZ; tgtG = angG;
  flushServos();
}

/* 记录当前笔尖位置为示教点 */
void teachRecord()
{
  if(teachCount >= TEACH_MAX)
  {
    Serial.println(F("→ 示教点已满（最多 8 个）"));
    return;
  }

  /* 用正解把当前舵机角换算成笔尖坐标 */
  float x, y, z;
  fk(angX, angY, angZ, x, y, z);

  /* 扣掉笔尖偏移 */
  x -= TIP_DX; y -= TIP_DY;

  /* 示教记录的是水平位置，纸面高度统一用 PEN_DOWN_Z */
  teachX[teachCount] = x;
  teachY[teachCount] = y;
  teachZ[teachCount] = PEN_DOWN_Z;
  teachCount++;

  Serial.print(F("→ 记录示教点 "));
  Serial.print(teachCount);
  Serial.print(F(": x="));
  Serial.print(x, 1);
  Serial.print(F(" y="));
  Serial.print(y, 1);
  Serial.print(F("  (当前 z="));
  Serial.print(z, 1);
  Serial.println(F(")"));
}

void teachClear()
{
  teachCount = 0;
  Serial.println(F("→ 示教点已清空"));
}

/* =====================================================================
 *  十二、串口指令
 * ===================================================================== */

void printHelp()
{
  Serial.println(F("L 直线   N/Z/V 字母   三角=Y"));
  Serial.println(F("F 五点折线   S 五点曲线"));
  Serial.println(F("R 进入示教   G 记录点   X 清空点"));
  Serial.println(F("P 暂停/继续   C 取消   M 回中   Q 状态   ? 帮助"));
}

void printStatus()
{
  Serial.print(F("模式:"));
  if(mode == MODE_IDLE)        Serial.print(F("待机"));
  else if(mode == MODE_MANUAL) Serial.print(F("手动"));
  else if(mode == MODE_TEACH)  Serial.print(F("示教"));
  else                         Serial.print(drawPaused ? F("绘制(暂停)") : F("绘制中"));

  float x, y, z;
  fk(angX, angY, angZ, x, y, z);
  Serial.print(F("  笔尖 x=")); Serial.print(x, 1);
  Serial.print(F(" y="));       Serial.print(y, 1);
  Serial.print(F(" z="));       Serial.print(z, 1);
  Serial.print(F("  示教点=")); Serial.print(teachCount);
  Serial.print(F("/"));         Serial.print(TEACH_MAX);
  Serial.print(F("  轨迹="));   Serial.print(drawCount);
  Serial.print(F("/"));         Serial.println(TRAJ_MAX);
}

void goHome()
{
  mode = MODE_IDLE;
  drawCancel = false;
  drawPaused = false;
  moveToStart(HOME_X, HOME_Y, HOME_Z, G_PEN);
  Serial.println(F("→ 回中"));
}

void serialPoll()
{
  while(Serial.available())
  {
    char c = (char)Serial.read();
    cmdLastMs = millis();

    if(c == '\r') continue;

    if(c == '\n')
    {
      cmdBuf[cmdLen] = '\0';

      if(!cmdOverflow && cmdLen > 0)
      {
        for(uint8_t i = 0; i < cmdLen; i++)
          if(cmdBuf[i] >= 'a' && cmdBuf[i] <= 'z') cmdBuf[i] -= 32;
        executeCommand();
      }
      cmdLen = 0;
      cmdOverflow = false;
      continue;
    }

    if(cmdLen < CMD_BUF - 1) cmdBuf[cmdLen++] = c;
    else                     cmdOverflow = true;
  }

  if(cmdLen > 0 && (millis() - cmdLastMs) > CMD_IDLE_MS)
  {
    cmdLen = 0;
    cmdOverflow = false;
  }
}

/* 执行 cmdBuf 里的指令（已转大写、已去行尾） */
void executeCommand()
{
  char c0 = cmdBuf[0];
  char c1 = cmdBuf[1];

  /* 任何时候都响应的指令 */
  if(c0 == 'M' && c1 == '\0') { goHome();             return; }
  if(c0 == 'Q' && c1 == '\0') { printStatus();        return; }
  if(c0 == '?' && c1 == '\0') { printHelp();          return; }
  if(c0 == 'P' && c1 == '\0')
  {
    if(mode == MODE_DRAW)
    {
      drawPaused = !drawPaused;
      Serial.println(drawPaused ? F("→ 已暂停（再按 D2 或发 P 继续）")
                                : F("→ 已继续，从暂停位置接续"));
    }
    else Serial.println(F("→ 当前没有绘图任务"));
    return;
  }
  if(c0 == 'C' && c1 == '\0')
  {
    if(mode == MODE_DRAW)
    {
      drawCancel = true;             // 由 drawTick 里安全地停下并抬笔
      Serial.println(F("→ 取消请求已提交"));
    }
    else Serial.println(F("→ 当前没有绘图任务"));
    return;
  }

  /* 示教相关 */
  if(c0 == 'R' && c1 == '\0')
  {
    if(mode == MODE_DRAW) { Serial.println(F("→ 绘图进行中，先按 C 取消")); return; }
    teachClear();
    mode = MODE_TEACH;
    Serial.println(F("==== 示教模式：摇杆摆到目标点，按 D4 或发 G 记录 ===="));
    return;
  }
  if(c0 == 'G' && c1 == '\0') { teachRecord(); return; }
  if(c0 == 'X' && c1 == '\0') { teachClear();  return; }

  /* 绘图任务：绘图进行中拒绝新的绘图指令 */
  if(mode == MODE_DRAW)
  {
    Serial.println(F("→ 忙碌中（P 暂停 / C 取消）"));
    return;
  }

  if(c0 == 'L' && c1 == '\0') { buildLine();        startDraw(); return; }
  if(c0 == 'N' && c1 == '\0') { buildShape(0);      startDraw(); return; }
  if(c0 == 'Z' && c1 == '\0') { buildShape(1);      startDraw(); return; }
  if(c0 == 'V' && c1 == '\0') { buildShape(2);      startDraw(); return; }
  if(c0 == 'Y' && c1 == '\0') { buildShape(3);      startDraw(); return; }
  if(c0 == 'F' && c1 == '\0') { buildPolyline();    startDraw(); return; }
  if(c0 == 'S' && c1 == '\0') { buildCurve();       startDraw(); return; }

  Serial.println(F("→ 未知指令，发 ? 看帮助"));
}

/* =====================================================================
 *  十三、按键
 * ===================================================================== */

void keyScan()
{
  static uint8_t raw[4]    = { HIGH, HIGH, HIGH, HIGH };
  static uint8_t stable[4] = { HIGH, HIGH, HIGH, HIGH };
  static unsigned long changeMs[4];

  for(int k = 0; k < 4; k++)
  {
    uint8_t now = digitalRead(keyPins[k]);
    if(now != raw[k]) { raw[k] = now; changeMs[k] = millis(); continue; }

    if(now != stable[k] && (millis() - changeMs[k]) > 25)
    {
      stable[k] = now;
      if(stable[k] != LOW) continue;

      /* D5 回中：无条件，兼急停 */
      if(k == 3) { goHome(); continue; }

      /* D2 暂停/继续 */
      if(k == 0)
      {
        if(mode == MODE_DRAW)
        {
          drawPaused = !drawPaused;
          Serial.println(drawPaused ? F("→ 已暂停") : F("→ 已继续"));
        }
        continue;
      }
      /* D3 取消 */
      if(k == 1)
      {
        if(mode == MODE_DRAW) { drawCancel = true; Serial.println(F("→ 取消")); }
        continue;
      }
      /* D4 记录示教点（在示教模式或手动模式下都可用） */
      if(k == 2)
      {
        if(mode != MODE_DRAW) teachRecord();
        continue;
      }
    }
  }
}

/* =====================================================================
 *  十四、setup / loop
 * ===================================================================== */

void setup()
{
  angX = HOME_X; angY = HOME_Y; angZ = HOME_Z; angG = G_PEN;
  lastX = angX;  lastY = angY;  lastZ = angZ;  lastG = angG;
  speedStep = 2;
  mode = MODE_IDLE;
  isMoving = false;
  arriveAt = 0;
  tgtX = angX; tgtY = angY; tgtZ = angZ; tgtG = angG;
  drawCount = 0; drawCursor = 0;
  teachCount = 0;
  cmdLen = 0; cmdOverflow = false; cmdLastMs = 0;

  Serial.begin(9600);

  for(int k = 0; k < 4; k++) pinMode(keyPins[k], INPUT_PULLUP);

  servoX.attach(9);
  servoY.attach(8);
  servoZ.attach(7);
  servoG.attach(6);

  servoX.write(angX);
  servoY.write(angY);
  servoZ.write(angZ);
  servoG.write(angG);
  delay(500);

  Serial.println(F("MeArm 任务五 轨迹绘制 就绪"));
  printHelp();
  printStatus();
}

void loop()
{
  serialPoll();
  keyScan();

  /* 绘制中不允许摇杆插手，其余模式摇杆始终可用（示教就靠它） */
  if(mode != MODE_DRAW)
  {
    if(!isMoving || mode == MODE_TEACH) joystickControl();
    else                                moveUpdate(250);
  }
  else
  {
    drawTick();
  }

  delay(20);
}