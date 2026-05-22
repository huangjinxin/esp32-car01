#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============ WiFi 热点配置 ============
const char* ssid = "ESP32-Car";
const char* password = "12345678";

// ============ L298N 电机引脚 ============
// IN1-IN4 分别接 GPIO15-GPIO18
const int PIN_IN1 = 15;
const int PIN_IN2 = 16;
const int PIN_IN3 = 17;
const int PIN_IN4 = 18;

// ============ 巡线传感器 (8 路) ============
// 测试接 4 路：OUT1-OUT4 → GPIO4-GPIO7
#define SENSOR_COUNT  4
const int sensorPins[SENSOR_COUNT] = { 4, 5, 6, 7 };
const char* sensorLabels[SENSOR_COUNT] = { "1", "2", "3", "4" };

// ============ 超声波传感器 (HC-SR04) ============
// Trig→GPIO8, Echo→GPIO9 （离得近且不与 USB 冲突）
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;
const float SOUND_SPEED = 0.034;  // 声速 cm/us

// 读取传感器状态 (0=白/无反光, 1=黑/有反光)
// 大多数巡线传感器是: LOW=检测到黑线, HIGH=白色地面
// 如果读数相反，把下面的 true 改成 false
static const bool SENSOR_ACTIVE_LOW = true;

void readSensors(bool* values) {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int raw = digitalRead(sensorPins[i]);
    values[i] = SENSOR_ACTIVE_LOW ? !raw : raw;
  }
}

// ============ PWM 速度控制 ============
// ENA/ENB 跳线短接时，用 IN 引脚 PWM 调速
#define PWM_FREQ     5000   // 5kHz
#define PWM_RES      8      // 8 位分辨率 (0-255)
#define MAX_SPEED    255
// 60% 速度 ≈ 153（L298N 需要足够电压才能转动）
int motorSpeed = 153;

// PWM 通道分配
const int CH_LF = 0;  // IN1 - 左前
const int CH_LB = 1;  // IN2 - 左后
const int CH_RF = 2;  // IN3 - 右前
const int CH_RB = 3;  // IN4 - 右后

// ============ Web 服务器 ============
WebServer server(80);

// ============ 电机控制函数（PWM 调速） ============
// in1-in4: 1=前进方向，0=停止/后退
// 实际写入 PWM 值：1 → motorSpeed，0 → 0
void motorWrite(int in1, int in2, int in3, int in4) {
  ledcWrite(CH_LF, in1 ? motorSpeed : 0);
  ledcWrite(CH_LB, in2 ? motorSpeed : 0);
  ledcWrite(CH_RF, in3 ? motorSpeed : 0);
  ledcWrite(CH_RB, in4 ? motorSpeed : 0);
}

// 电机方向修正说明：
//   - 右侧电机 (IN3/IN4) 接线极性相反 → IN3↔IN4 对调
//   - 整体方向反了 → IN1↔IN2 也对调（即前进↔后退互换）
// 如果重新接线后方向不对，整体交换 (HIGH,LOW)↔(LOW,HIGH) 即可
void motorForward() {
  motorWrite(LOW, HIGH, HIGH, LOW);   // 原: H,L, L,H → 整体反向调整
}

void motorBackward() {
  motorWrite(HIGH, LOW, LOW, HIGH);   // 原: L,H, H,L → 整体反向调整
}

void motorTurnLeft() {
  motorWrite(HIGH, LOW, HIGH, LOW);   // 原: L,H, L,H → 整体反向调整
}

void motorTurnRight() {
  motorWrite(LOW, HIGH, LOW, HIGH);   // 原: H,L, H,L → 整体反向调整
}

void motorStop() {
  motorWrite(LOW, LOW, LOW, LOW);
}

// ============ 超声波测距 ============
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // pulseIn 超时 30ms ≈ 测距上限约 5m
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;  // 超时 / 无回波

  // 距离 = 时间 × 声速 / 2 (来回)
  float distance = duration * SOUND_SPEED / 2.0;
  return distance;
}

// 传感器状态缓存（供 web 页面轮询）
bool sensorValues[SENSOR_COUNT] = { false };
unsigned long lastSensorPrint = 0;

// 当前电机命令（供 loop 安全监控用）
String currentCmd = "stop";
unsigned long lastSafetyCheck = 0;
const float OBSTACLE_THRESHOLD = 20.0;  // 避障阈值 cm
unsigned long evacuateStart = 0;        // 撤离开始时间戳（触发的时刻）
const unsigned long EVACUATE_DURATION = 1000;  // 撤离（反向跑）时长 ms

// 自动模式（避障模式）：5秒定时
unsigned long autoMoveStart = 0;
const unsigned long AUTO_MOVE_DURATION = 5000;

// 巡线模式
bool lineFollowing = false;
const int LINE_FOLLOW_SPEED = 100;  // 巡线速度（慢速，0-255）
int savedMotorSpeed = 153;          // 保存的原速度
unsigned long lastLineCheck = 0;

// ============ HTML 控制页面（嵌入在代码中） ============
const char html_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>ESP32 小车控制</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body {
  font-family: -apple-system, 'Segoe UI', sans-serif;
  background: #f0f2f5;
  display: flex;
  justify-content: center;
  padding: 12px;
  min-height: 100vh;
  touch-action: none;
  user-select: none;
  -webkit-user-select: none;
}
.container {
  max-width: 420px;
  width: 100%;
}

/* ===== 卡片通用 ===== */
.card {
  background: #fff;
  border-radius: 16px;
  padding: 16px;
  margin-bottom: 12px;
  box-shadow: 0 2px 12px rgba(0,0,0,0.08);
}
.card-title {
  font-size: 14px;
  font-weight: 700;
  color: #1e293b;
  margin-bottom: 10px;
  display: flex;
  align-items: center;
  gap: 6px;
}

/* ===== 状态卡片 ===== */
.status-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
}
.status-item {
  background: #f8fafc;
  border-radius: 12px;
  padding: 10px;
  text-align: center;
}
.status-item .label {
  font-size: 11px;
  color: #94a3b8;
  margin-bottom: 3px;
}
.status-item .value {
  font-size: 22px;
  font-weight: 800;
}
.val-distance { color: #10b981; }
.val-distance.null { color: #94a3b8; }
.val-state { font-size: 13px !important; }
.state-stop { color: #94a3b8; }
.state-moving { color: #3b82f6; }
.state-evacuating { color: #f59e0b; }
.state-blocked { color: #ef4444; }
.state-line { color: #8b5cf6; }
.sensor-mini {
  display: flex;
  justify-content: center;
  gap: 5px;
  margin-top: 5px;
}
.sensor-mini .dot {
  width: 10px; height: 10px;
  border-radius: 50%;
  background: #e2e8f0;
  transition: all 0.1s;
}
.sensor-mini .dot.on {
  background: #3b82f6;
  box-shadow: 0 0 6px rgba(59,130,246,0.5);
}

/* ===== 手动控制 ===== */
.dpad {
  display: grid;
  grid-template-columns: 64px 64px 64px;
  grid-template-rows: 64px 64px 64px;
  gap: 5px;
  justify-content: center;
  margin-bottom: 8px;
}
.dpad-btn {
  border: none;
  border-radius: 14px;
  font-size: 22px;
  cursor: pointer;
  background: #1e293b;
  color: #fff;
  box-shadow: 0 3px 10px rgba(0,0,0,0.15);
  transition: all 0.08s;
  -webkit-tap-highlight-color: transparent;
  display: flex;
  align-items: center;
  justify-content: center;
}
.dpad-btn:active, .dpad-btn.active {
  background: #ef4444;
  transform: scale(0.9);
}
.btn-up    { grid-column: 2; grid-row: 1; }
.btn-left  { grid-column: 1; grid-row: 2; }
.btn-stop  { grid-column: 2; grid-row: 2; background: #475569; font-size: 11px; font-weight: 700; color: #ef4444; }
.btn-right { grid-column: 3; grid-row: 2; }
.btn-down  { grid-column: 2; grid-row: 3; }
.ctrl-hint {
  text-align: center;
  font-size: 11px;
  color: #94a3b8;
}
.ctrl-hint span {
  display: inline-block;
  background: #f1f5f9;
  padding: 3px 8px;
  border-radius: 6px;
  margin: 2px;
}

/* ===== 避障模式 ===== */
.auto-row {
  display: flex;
  justify-content: center;
  gap: 8px;
  margin: 8px 0;
}
.auto-btn {
  border: none;
  border-radius: 10px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  padding: 10px 16px;
  background: #f1f5f9;
  color: #1e293b;
  transition: all 0.08s;
  -webkit-tap-highlight-color: transparent;
  flex: 1;
  max-width: 120px;
}
.auto-btn:active, .auto-btn.active {
  background: #ef4444;
  color: #fff;
  transform: scale(0.93);
}
.auto-btn.a-stop {
  background: #fee2e2;
  color: #ef4444;
}
.auto-btn.a-stop:active {
  background: #ef4444;
  color: #fff;
}
.auto-timer {
  text-align: center;
  font-size: 18px;
  font-weight: 700;
  color: #f59e0b;
  min-height: 26px;
}
.auto-status {
  text-align: center;
  font-size: 12px;
  color: #94a3b8;
  margin-top: 2px;
}

/* ===== 巡线模式 ===== */
.line-row {
  display: flex;
  justify-content: center;
  gap: 10px;
  margin: 8px 0;
}
.line-btn {
  border: none;
  border-radius: 10px;
  font-size: 15px;
  font-weight: 700;
  cursor: pointer;
  padding: 12px 24px;
  transition: all 0.08s;
  -webkit-tap-highlight-color: transparent;
  flex: 1;
  max-width: 160px;
}
.ln-start { background: #8b5cf6; color: #fff; }
.ln-start:active, .ln-start.active { background: #7c3aed; transform: scale(0.93); }
.ln-stop { background: #f1f5f9; color: #64748b; }
.ln-stop:active { background: #e2e8f0; transform: scale(0.93); }
.line-status {
  text-align: center;
  font-size: 12px;
  color: #94a3b8;
  margin-top: 6px;
}

/* =========================================================
   iPad / 平板响应式适配
   ========================================================= */
@media (min-width: 600px) {
  body { padding: 20px; }
  .container { max-width: 700px; }

  .card { padding: 22px; margin-bottom: 18px; border-radius: 20px; }
  .card-title { font-size: 17px; margin-bottom: 14px; }

  /* 状态卡：更大更清晰 */
  .status-grid { gap: 14px; }
  .status-item { padding: 16px; border-radius: 14px; }
  .status-item .label { font-size: 13px; }
  .status-item .value { font-size: 28px; }
  .val-state { font-size: 16px !important; }
  .sensor-mini .dot { width: 14px; height: 14px; }
  .sensor-mini { gap: 8px; }

  /* 底部三张操作卡：2列布局 */
  .card-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 18px;
  }

  /* 手动控制：方向键更大 */
  .dpad {
    grid-template-columns: 80px 80px 80px;
    grid-template-rows: 80px 80px 80px;
    gap: 8px;
  }
  .dpad-btn { font-size: 28px; border-radius: 18px; }
  .btn-stop { font-size: 14px; }

  /* 避障按钮 */
  .auto-btn { font-size: 16px; padding: 14px 20px; max-width: 140px; }
  .auto-timer { font-size: 22px; }

  /* 巡线按钮 */
  .line-btn { font-size: 17px; padding: 16px 32px; max-width: 200px; }
  .line-status { font-size: 14px; }
}

@media (min-width: 900px) {
  .container { max-width: 900px; }
  .card-grid {
    grid-template-columns: 1fr 1fr 1fr;
  }
}
</style>
</head>
<body>
<div class="container">

  <!-- ===== 状态卡片 ===== -->
  <div class="card">
    <div class="card-title">📊 运行状态</div>
    <div class="status-grid">
      <div class="status-item">
        <div class="label">📡 超声波距离</div>
        <div class="value val-distance" id="distVal">---</div>
      </div>
      <div class="status-item">
        <div class="label">🔲 巡线传感器</div>
        <div class="sensor-mini" id="sensorMini">
          <span class="dot" data-i="0"></span>
          <span class="dot" data-i="1"></span>
          <span class="dot" data-i="2"></span>
          <span class="dot" data-i="3"></span>
        </div>
      </div>
    </div>
    <div style="display:flex;justify-content:space-between;margin-top:8px;gap:8px">
      <div class="status-item" style="flex:1">
        <div class="label">🔄 当前状态</div>
        <div class="value val-state state-stop" id="stateVal">已停止</div>
      </div>
      <div class="status-item" style="flex:1">
        <div class="label">⏱ 倒计时</div>
        <div class="value val-distance" id="timerVal" style="font-size:20px">0.0s</div>
      </div>
    </div>
  </div>

  <!-- ===== 操作卡片网格（iPad 2列，手机1列） ===== -->
  <div class="card-grid">

  <!-- ===== 手动控制卡片 ===== -->
  <div class="card">
    <div class="card-title">🎮 手动控制</div>
    <div class="dpad">
      <button class="dpad-btn btn-up"    data-action="forward">▲</button>
      <button class="dpad-btn btn-left"  data-action="left">◀</button>
      <button class="dpad-btn btn-stop"  data-action="stop">STOP</button>
      <button class="dpad-btn btn-right" data-action="right">▶</button>
      <button class="dpad-btn btn-down"  data-action="backward">▼</button>
    </div>
    <div class="ctrl-hint">
      <span>按住=移动</span>
      <span>松开=停止</span>
      <span>WASD/方向键</span>
    </div>
  </div>

  <!-- ===== 避障模式卡片 ===== -->
  <div class="card">
    <div class="card-title">🛡️ 避障模式（5秒定时）</div>
    <div class="auto-row">
      <button class="auto-btn" data-auto="forward">▲ 前进</button>
      <button class="auto-btn a-stop" data-auto="stop">■ STOP</button>
      <button class="auto-btn" data-auto="backward">▼ 后退</button>
    </div>
    <div class="auto-timer" id="autoTimer">⏱ 0.0s</div>
    <div class="auto-status" id="autoStatus">点击按钮启动</div>
  </div>

  <!-- ===== 巡线模式卡片 ===== -->
  <div class="card">
    <div class="card-title">🚗 巡线模式</div>
    <div class="line-row">
      <button class="line-btn ln-start" id="lnStartBtn">▶ 开始巡线</button>
      <button class="line-btn ln-stop"  id="lnStopBtn">■ 停止</button>
    </div>
    <div class="line-status" id="lineStatus">就绪 — 将小车放在黑线上</div>
  </div>

  </div><!-- /card-grid -->

</div>

<script>
(function() {
  // ========= 手动控制 =========
  let pointerDownAction = null;
  const stateVal = document.getElementById('stateVal');
  const timerVal = document.getElementById('timerVal');
  const distVal = document.getElementById('distVal');
  const dpadBtns = document.querySelectorAll('.dpad-btn');

  function sendAction(action) {
    fetch('/action?cmd=' + action)
      .then(r => r.text())
      .then(t => {
        if (t === 'BLOCKED') {
          stateVal.textContent = '⚠ 遇障撤离';
          stateVal.className = 'value val-state state-evacuating';
          pointerDownAction = null;
        }
      })
      .catch(() => {});
  }

  dpadBtns.forEach(btn => {
    btn.addEventListener('pointerdown', function(e) {
      e.preventDefault();
      const a = this.dataset.action;
      if (a !== 'stop') {
        pointerDownAction = a;
        dpadBtns.forEach(b => b.classList.remove('active'));
        this.classList.add('active');
        sendAction(a);
      } else {
        pointerDownAction = null;
        dpadBtns.forEach(b => b.classList.remove('active'));
        sendAction('stop');
      }
    });
  });
  document.addEventListener('pointerup', function() {
    if (pointerDownAction) {
      pointerDownAction = null;
      dpadBtns.forEach(b => b.classList.remove('active'));
      sendAction('stop');
    }
  });
  document.addEventListener('pointercancel', function() {
    if (pointerDownAction) {
      pointerDownAction = null;
      dpadBtns.forEach(b => b.classList.remove('active'));
      sendAction('stop');
    }
  });

  // 键盘
  const keyMap = {
    'ArrowUp':'forward','w':'forward','W':'forward',
    'ArrowDown':'backward','s':'backward','S':'backward',
    'ArrowLeft':'left','a':'left','A':'left',
    'ArrowRight':'right','d':'right','D':'right',
    ' ':'stop'
  };
  document.addEventListener('keydown', function(e) {
    const a = keyMap[e.key];
    if (a && a !== 'stop') { e.preventDefault();
      if (pointerDownAction !== a) {
        pointerDownAction = a;
        dpadBtns.forEach(b => b.classList.remove('active'));
        const el = document.querySelector(`[data-action="${a}"]`);
        if (el) el.classList.add('active');
        sendAction(a);
      }
    } else if (a === 'stop') { e.preventDefault();
      pointerDownAction = null;
      dpadBtns.forEach(b => b.classList.remove('active'));
      sendAction('stop');
    }
  });
  document.addEventListener('keyup', function(e) {
    const a = keyMap[e.key];
    if (a && a !== 'stop') { e.preventDefault();
      if (pointerDownAction === a) {
        pointerDownAction = null;
        dpadBtns.forEach(b => b.classList.remove('active'));
        sendAction('stop');
      }
    }
  });

  // ========= 避障模式按钮 =========
  const autoBtns = document.querySelectorAll('.auto-btn');
  const autoTimer = document.getElementById('autoTimer');
  const autoStatus = document.getElementById('autoStatus');
  autoBtns.forEach(btn => {
    btn.addEventListener('click', function(e) {
      e.preventDefault();
      const a = this.dataset.auto;
      const m = { 'forward':'auto_forward','backward':'auto_backward','stop':'auto_stop' };
      const fc = m[a] || 'auto_stop';
      autoBtns.forEach(b => b.classList.remove('active'));
      if (a !== 'stop') this.classList.add('active');
      fetch('/action?cmd=' + fc).then(r => r.text()).then(t => {
        if (t === 'BLOCKED') {
          autoTimer.textContent = '⛔ 遇障撤离中';
          autoStatus.innerHTML = '<span style="color:#f59e0b">⚠ 自动撤离</span>';
          autoBtns.forEach(b => b.classList.remove('active'));
        }
      }).catch(() => {});
    });
  });

  // ========= 巡线模式按钮 =========
  const lnStart = document.getElementById('lnStartBtn');
  const lnStop = document.getElementById('lnStopBtn');
  const lineStatus = document.getElementById('lineStatus');
  lnStart.addEventListener('click', function() {
    fetch('/line_action?cmd=start').then(r => r.text()).then(t => {
      if (t === 'LINE_START') {
        lnStart.classList.add('active');
        lineStatus.innerHTML = '📍 <strong>巡线中</strong> — 沿黑线行驶';
      }
    }).catch(() => {});
  });
  lnStop.addEventListener('click', function() {
    fetch('/line_action?cmd=stop').then(r => r.text()).then(t => {
      if (t === 'LINE_STOP') {
        lnStart.classList.remove('active');
        lineStatus.textContent = '已停止 — 将小车放在黑线上';
      }
    }).catch(() => {});
  });

  // ========= 轮询：传感器（每 100ms） =========
  const dots = document.querySelectorAll('#sensorMini .dot');
  setInterval(function() {
    fetch('/sensors').then(r => r.json()).then(d => {
      for (let i = 0; i < d.length && i < dots.length; i++) {
        dots[i].classList.toggle('on', !!d[i]);
      }
    }).catch(() => {});
  }, 100);

  // ========= 轮询：距离 + 状态（每 200ms） =========
  setInterval(function() {
    fetch('/distance').then(r => r.json()).then(d => {
      // 距离
      if (d.distance !== null) {
        distVal.textContent = d.distance.toFixed(1) + ' cm';
        distVal.className = 'value val-distance';
      } else {
        distVal.textContent = '--- cm';
        distVal.className = 'value val-distance null';
      }

      // 状态
      const cmd = d.cmd || 'stop';
      const remain = d.remaining || 0;

      if (d.lineFollowing) {
        stateVal.textContent = '📍 巡线中';
        stateVal.className = 'value val-state state-line';
        timerVal.textContent = '∞';
      } else if (cmd === 'auto_forward') {
        stateVal.textContent = '▶ 自动前进';
        stateVal.className = 'value val-state state-moving';
        timerVal.textContent = remain.toFixed(1) + 's';
      } else if (cmd === 'auto_backward') {
        stateVal.textContent = '◀ 自动后退';
        stateVal.className = 'value val-state state-moving';
        timerVal.textContent = remain.toFixed(1) + 's';
      } else if (cmd === 'evacuating') {
        stateVal.textContent = '⚠ 撤离中';
        stateVal.className = 'value val-state state-evacuating';
        timerVal.textContent = remain.toFixed(1) + 's';
      } else if (cmd === 'forward') {
        stateVal.textContent = '▲ 前进';
        stateVal.className = 'value val-state state-moving';
        timerVal.textContent = '手动';
      } else if (cmd === 'backward') {
        stateVal.textContent = '▼ 后退';
        stateVal.className = 'value val-state state-moving';
        timerVal.textContent = '手动';
      } else if (cmd === 'left') {
        stateVal.textContent = '◀ 左转';
        stateVal.className = 'value val-state state-moving';
        timerVal.textContent = '手动';
      } else if (cmd === 'right') {
        stateVal.textContent = '▶ 右转';
        stateVal.className = 'value val-state state-moving';
        timerVal.textContent = '手动';
      } else {
        stateVal.textContent = '已停止';
        stateVal.className = 'value val-state state-stop';
        timerVal.textContent = '0.0s';
      }

      // 避障卡片同步
      if (d.lineFollowing) {
        autoTimer.textContent = '⏱ 0.0s';
        autoStatus.innerHTML = '<span style="color:#8b5cf6">巡线模式已启动</span>';
      } else if (cmd === 'auto_forward') {
        autoTimer.textContent = '⏱ ' + remain.toFixed(1) + 's';
        autoStatus.innerHTML = '<span style="color:#3b82f6">▶ 自动前进中</span>';
      } else if (cmd === 'auto_backward') {
        autoTimer.textContent = '⏱ ' + remain.toFixed(1) + 's';
        autoStatus.innerHTML = '<span style="color:#3b82f6">◀ 自动后退中</span>';
      } else if (cmd === 'evacuating') {
        autoTimer.textContent = '⛔ 撤离 ' + remain.toFixed(1) + 's';
        autoStatus.innerHTML = '<span style="color:#f59e0b">⚠ 遇障撤离</span>';
      } else {
        autoTimer.textContent = '⏱ 0.0s';
        autoStatus.innerHTML = '就绪';
      }
    }).catch(() => {});
  }, 200);

})();
</script>
</body>
</html>
)rawliteral";

// ============ 巡线模式 API ============
void handleLineAction() {
  String cmd = server.arg("cmd");

  if (cmd == "start") {
    // 停掉当前任何动作
    motorStop();
    currentCmd = "stop";
    // 保存原速度，切换成慢速
    savedMotorSpeed = motorSpeed;
    motorSpeed = LINE_FOLLOW_SPEED;
    // 启动巡线
    lineFollowing = true;
    Serial.printf(">> 巡线模式启动（速度=%d）\n", LINE_FOLLOW_SPEED);
    server.send(200, "text/plain", "LINE_START");
  } else if (cmd == "stop") {
    lineFollowing = false;
    motorSpeed = savedMotorSpeed;
    motorStop();
    currentCmd = "stop";
    Serial.println(">> 巡线模式停止");
    server.send(200, "text/plain", "LINE_STOP");
  } else {
    server.send(400, "text/plain", "BAD");
  }
}

// ============ HTTP 请求处理 ============
void handleRoot() {
  server.send(200, "text/html", html_page);
}

void handleAction() {
  String cmd = server.arg("cmd");

  // 如果正在巡线，先退出巡线模式
  if (lineFollowing && cmd != "stop") {
    lineFollowing = false;
    motorSpeed = savedMotorSpeed;
  }

  // 任何方向的移动，先停掉当前动作
  if (cmd != "stop") {
    // 如果正在撤离或自动模式，先中止
    if (currentCmd == "evacuating" || currentCmd == "auto_forward" || currentCmd == "auto_backward") {
      motorStop();
      currentCmd = "stop";
    }
  }

  // ========== 避障拦截：后退时检查 ==========
  if (cmd == "backward" || cmd == "auto_backward") {
    float d = measureDistance();
    if (d > 0 && d < OBSTACLE_THRESHOLD) {
      // 触发避障 → 停止 → 反向跑（前进）撤离
      motorForward();                          // 反向跑 = 前进
      currentCmd = "evacuating";
      evacuateStart = millis();
      server.send(403, "text/plain", "BLOCKED");
      Serial.printf(">> 避障！距离=%.1fcm < %.0fcm，反向撤离 %dms\n",
                    d, OBSTACLE_THRESHOLD, EVACUATE_DURATION);
      return;
    }
  }

  // ========== 执行命令 ==========
  if (cmd == "forward") {
    motorForward();
    currentCmd = "forward";
    Serial.println(">> 前进");
  } else if (cmd == "backward") {
    motorBackward();
    currentCmd = "backward";
    Serial.println(">> 后退");
  } else if (cmd == "left") {
    motorTurnLeft();
    currentCmd = "left";
    Serial.println(">> 左转");
  } else if (cmd == "right") {
    motorTurnRight();
    currentCmd = "right";
    Serial.println(">> 右转");
  } else if (cmd == "stop" || cmd == "auto_stop") {
    motorStop();
    currentCmd = "stop";
    Serial.println(">> 停止");
  } else if (cmd == "auto_forward") {
    motorForward();
    currentCmd = "auto_forward";
    autoMoveStart = millis();
    Serial.println(">> 自动前进（5秒）");
  } else if (cmd == "auto_backward") {
    motorBackward();
    currentCmd = "auto_backward";
    autoMoveStart = millis();
    Serial.println(">> 自动后退（5秒）");
  } else {
    server.send(400, "text/plain", "BAD");
    return;
  }

  server.send(200, "text/plain", cmd);
}

// ============ 传感器 API ============
void handleSensors() {
  readSensors(sensorValues);
  // 返回 JSON 数组，如: [true,false,true,false]
  String json = "[";
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (i > 0) json += ",";
    json += sensorValues[i] ? "true" : "false";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// ============ 超声波 API + 全状态 ============
void handleDistance() {
  float d = measureDistance();
  // 计算倒计时剩余秒数
  float remaining = 0;
  String modeDesc = currentCmd;  // 模式描述

  if (currentCmd == "auto_forward" || currentCmd == "auto_backward") {
    unsigned long elapsed = millis() - autoMoveStart;
    if (elapsed < AUTO_MOVE_DURATION)
      remaining = (AUTO_MOVE_DURATION - elapsed) / 1000.0;
  } else if (currentCmd == "evacuating") {
    unsigned long elapsed = millis() - evacuateStart;
    if (elapsed < EVACUATE_DURATION)
      remaining = (EVACUATE_DURATION - elapsed) / 1000.0;
  } else if (currentCmd == "stop" && lineFollowing) {
    modeDesc = "line_following";
  }

  String json = "{\"distance\":";
  if (d < 0) json += "null";
  else       json += String(d, 1);
  json += ",\"cmd\":\"" + modeDesc + "\"";
  json += ",\"remaining\":" + String(remaining, 1);
  json += ",\"lineFollowing\":" + String(lineFollowing ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ============ setup / loop ============
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32 智能小车启动 ===");

  // 初始化 PWM 通道（替代普通 digitalWrite，用于调速）
  ledcSetup(CH_LF, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_IN1, CH_LF);
  ledcSetup(CH_LB, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_IN2, CH_LB);
  ledcSetup(CH_RF, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_IN3, CH_RF);
  ledcSetup(CH_RB, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_IN4, CH_RB);
  motorStop();

  // 初始化巡线传感器引脚
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  Serial.println("巡线传感器已就绪 (OUT1-OUT4 → GPIO4-GPIO7)");

  // 初始化超声波传感器引脚
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.println("超声波传感器已就绪 (Trig→GPIO8, Echo→GPIO9)");

  // 启动 WiFi 热点
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);  // 关闭省电模式，提高稳定性

  // 显式配置 AP 的 IP 和 DHCP 范围
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  WiFi.softAP(ssid, password);
  delay(500);  // 等待 WiFi 完全初始化

  Serial.print("WiFi 热点名称: ");
  Serial.println(ssid);
  Serial.print("连接密码: ");
  Serial.println(password);
  if (WiFi.softAPIP().toString() != "0.0.0.0") {
    Serial.print("控制地址: http://");
    Serial.println(WiFi.softAPIP());
    Serial.print("已连接设备数: ");
    Serial.println(WiFi.softAPgetStationNum());
  } else {
    Serial.println("⚠️ WiFi 热点启动失败！请检查板子状态");
  }

  // 配置 Web 路由
  server.on("/", handleRoot);
  server.on("/action", handleAction);
  server.on("/sensors", handleSensors);
  server.on("/distance", handleDistance);
  server.on("/line_action", handleLineAction);
  server.begin();
  Serial.println("Web 服务器已启动，等待浏览器控制...\n");
}

void loop() {
  server.handleClient();

  // ============ 实时状态机（每 30ms） ============
  if (millis() - lastSafetyCheck > 30) {
    lastSafetyCheck = millis();

    // ----- 巡线模式 -----
    if (lineFollowing) {
      // 每 50ms 做一次巡线控制（与 30ms 错开，用单独计时）
      if (millis() - lastLineCheck > 50) {
        lastLineCheck = millis();
        bool s[4];
        readSensors(s);
        // s[0]=GPIO4=OUT3(左2), s[1]=GPIO5=OUT4(左1)
        // s[2]=GPIO6=OUT5(右1), s[3]=GPIO7=OUT6(右2)

        // 计算偏差: 左负右正
        int error = 0;
        if (s[0]) error -= 3;  // 左外
        if (s[1]) error -= 1;  // 左内
        if (s[2]) error += 1;  // 右内
        if (s[3]) error += 3;  // 右外

        if (error == 0) {
          // 没有检测到黑线 → 停止
          motorStop();
        } else if (error < 0) {
          // 偏左 → 右转修正
          motorTurnRight();
        } else {
          // 偏右 → 左转修正
          motorTurnLeft();
        }
      }
    }
    // ----- 撤离状态：正在反向跑（远离障碍物） -----
    else if (currentCmd == "evacuating") {
      if (millis() - evacuateStart >= EVACUATE_DURATION) {
        motorStop();
        currentCmd = "stop";
        Serial.printf(">> 撤离完成（%dms），已停止\n", EVACUATE_DURATION);
      }
    }
    // ----- 自动模式：5秒定时 -----
    else if (currentCmd == "auto_forward" || currentCmd == "auto_backward") {
      // 自动后退时检查后方障碍物
      if (currentCmd == "auto_backward") {
        float d = measureDistance();
        if (d > 0 && d < OBSTACLE_THRESHOLD) {
          // 触发 → 反向跑（前进）撤离
          motorForward();
          currentCmd = "evacuating";
          evacuateStart = millis();
          Serial.printf(">> 自动后退遇障(%.1fcm)，撤离 %dms\n", d, EVACUATE_DURATION);
        }
      }
      // 检查 5 秒是否到
      if (currentCmd != "evacuating" && millis() - autoMoveStart >= AUTO_MOVE_DURATION) {
        motorStop();
        currentCmd = "stop";
        Serial.printf(">> 自动移动结束（%d秒）\n", AUTO_MOVE_DURATION / 1000);
      }
    }
    // ----- 手动后退：实时避障 -----
    else if (currentCmd == "backward") {
      float d = measureDistance();
      if (d > 0 && d < OBSTACLE_THRESHOLD) {
        motorForward();  // 反向跑 = 前进
        currentCmd = "evacuating";
        evacuateStart = millis();
        Serial.printf(">> 后退遇障(%.1fcm)，撤离 %dms\n", d, EVACUATE_DURATION);
      }
    }
  }

  // 定时读取传感器并打印到串口（每 500ms）
  if (millis() - lastSensorPrint > 500) {
    lastSensorPrint = millis();
    readSensors(sensorValues);

    Serial.print("传感器: ");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensorValues[i] ? "■ " : "_ ");
    }
    float d = measureDistance();
    Serial.printf(" | 距离: ");
    if (d < 0) Serial.print("---");
    else Serial.printf("%.1f", d);

    // 显示倒计时（自动模式 / 撤离模式）
    if (currentCmd == "auto_forward" || currentCmd == "auto_backward") {
      unsigned long elapsed = millis() - autoMoveStart;
      if (elapsed < AUTO_MOVE_DURATION)
        Serial.printf(" cm | 倒计时: %.1fs", (AUTO_MOVE_DURATION - elapsed) / 1000.0);
      else
        Serial.print(" cm | 倒计时: 0.0s");
    } else if (currentCmd == "evacuating") {
      unsigned long elapsed = millis() - evacuateStart;
      if (elapsed < EVACUATE_DURATION)
        Serial.printf(" cm | 撤离中: %.1fs", (EVACUATE_DURATION - elapsed) / 1000.0);
      else
        Serial.print(" cm | 撤离中: 0.0s");
    }
    if (lineFollowing) Serial.print(" cm | 巡线中");
    Serial.printf(" | 状态: %s\n", currentCmd.c_str());
  }
}
