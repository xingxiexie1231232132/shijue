'''
pid.py
======
🎯 功能：PID 控制器 — 把"误差"变成"控制量"。

   输入：error（比如巡线时线偏离画面中线的像素数）
   输出：一个数值，直接喂给舵机/电机驱小车回正

🧠 三个字母的含义（用开车类比）：
   P（比例） = 现在偏离了多少   → 现在就打方向盘
   I（积分） = 偏离了多久       → 一直偏就多打一点
   D（微分） = 偏离速度有多快   → 快冲出去了提前松方向盘

🛡️ 保护机制：
   · 积分限幅 — 防止 I 值无限累加（积分饱和）
   · 微分低通 — 过滤摄像头噪声，D 不抖动

硬件平台：MaixCamPro (Sipeed) / 通用 Python
依赖：    无硬件依赖，仅需 time 和 math 标准库
运行方式：作为模块被 main.py 导入使用

🧮 PID 公式：
    output = (Kp×error + Ki×∫error·dt + Kd×de/dt) × scaler

使用示例：
    from pid import PID
    pid = PID(p=1.1, i=0.8, d=0.01, imax=40)
    while True:
        error = target - measure        # 计算误差
        output = pid.get_pid(error, 1)  # 获取 PID 输出
        # 用 output 控制电机/舵机

参考：源自 OpenMV 巡线项目 pid.py (2021)
转换：pyb.millis → time.ticks_ms，其余逻辑不变
日期：2026-06-12
'''

from maix.time import ticks_ms     # ⏱️ MaixPy 毫秒计时（原 OpenMV 用 pyb.millis）
from math import pi, isnan         # 🧮 π 用于 RC 计算，isnan 判断微分是否初始化


# ╔══════════════════════════════════════════╗
# ║  1. PID 控制器类                          ║
# ╚══════════════════════════════════════════╝

class PID:
    '''
    🎯 离散 PID 控制器 — 把"误差"转换成"控制量"的数学工具。

    🧠 一句话理解 PID：
        P（比例）= 现在：误差大 → 用力纠正
        I（积分）= 过去：误差一直存在 → 越积越狠，消除残留
        D（微分）= 未来：误差变化快 → 提前刹车，防止冲过头

    🚗 类比开车：
        P = 看到离车道线近了 → 打方向盘
        I = 方向盘一直偏右 → 慢慢往左修正
        D = 车头正在快速偏转 → 松一点方向盘，防止甩尾

    🛡️ 额外保护：
        · 积分限幅（imax）：防止 I 项无限累积（积分饱和）
        · 微分低通滤波（20Hz）：过滤高频噪声，D 项不抖动

    Args:
        p:    比例系数 Kp。调大 → 响应快但易震荡
        i:    积分系数 Ki。调大 → 稳态误差小但易过冲
        d:    微分系数 Kd。调大 → 抑制震荡但易放大噪声
        imax: 积分上限。建议 = 期望输出的 30%~50%

    Example:
        pid = PID(p=1.1, i=0.8, d=0.01, imax=40)
        output = pid.get_pid(error, scaler=1.0)
    '''

    def __init__(self, p=0, i=0, d=0, imax=0):
        # ── 用户可调参数 ──
        self._kp = float(p)                     # 🔢 Kp — 比例系数（现在）
        self._ki = float(i)                     # 🔢 Ki — 积分系数（过去）
        self._kd = float(d)                     # 🔢 Kd — 微分系数（未来）
        self._imax = abs(imax)                  # 🛡️ 积分限幅，取绝对值防负值

        # ── 内部状态（跨帧记忆）──
        self._integrator = 0                    # 📦 积分累加器，累积历史误差
        self._last_error = 0                    # 📍 上一帧的误差，用于算变化率
        self._last_derivative = float('nan')    # 📍 上一帧微分值，nan=未初始化
        self._last_t = 0                        # ⏱️ 上次调用的时间戳（毫秒）

        # ── D 项低通滤波器（EMA 离散化模拟 RC 电路）──
        #
        # 🎯 目的：摄像头像素抖动会产生虚假高频微分 → D 项乱跳 → 舵机抽搐。
        #   低通滤波只让 < fc 的平滑变化通过，高频毛刺全拦掉。
        #
        # 🧮 两步推导：
        #   Step 1 — RC 电路原型：截止频率 fc = 1/(2π·RC)，设 fc=20Hz
        #            → RC = 1/(2π × 20) ≈ 0.00796 秒
        #   Step 2 — EMA 离散化：new = old + α×(raw − old)，α = dt/(RC+dt)
        #           等价写法 new = α×raw + (1−α)×old（新旧加权平均）
        #    30fps 时 dt≈0.033 → α≈0.80，尖峰被压到 80%
        #
        # 💡 调参：RC↑ → α↓ → 滤波强 → D 平滑但滞后
        #          RC↓ → α↑ → 滤波弱 → D 灵敏但易抖
        self._RC = 1 / (2 * pi * 20)


    # ── 1.1 计算 PID 输出 ──────────────────────

    def get_pid(self, error, scaler):
        '''
        🧮 每帧调用一次，把误差转为控制量。

        📐 输出公式：output = scaler×(Kp·error + Kd·filtered_derivative) + integrator
           I 项累加：integrator += Ki × error × scaler × dt（累加时已乘 scaler）

        Args:
            error:  当前误差（目标 − 实际），如画面中线 − 巡线中心。
            scaler: 输出缩放系数，1.0=不缩放，<1 更柔和，>1 更激进。

        Returns:
            float: 控制输出，直接交给舵机/电机。
        '''

        # ── 步骤 1：⏱️ 计算时间间隔 dt ──
        #   🧠 I 项 = 误差×时间 → dt 决定"攒了多少"
        #   🧠 D 项 = 误差变化÷时间 → dt 决定"变化多快"
        tnow = ticks_ms()                       # ⏱️ 当前毫秒时间戳
        dt = tnow - self._last_t                # 📏 距上次调用的毫秒数

        # 🛡️ 首次调用 / 间隔 >1 秒 / ticks_ms() 溢出回绕 → 复位积分。
        #    dt 置 0 → 本次只算 P，跳过依赖时间差的 I 和 D。
        #
        # 🐛 溢出回绕（旧版无防护）：
        #    ticks_ms() 返回一个无符号整数，从 0 开始每毫秒 +1。
        #    MicroPython 32 位环境的最大值 ≈ 2³² − 1 ≈ 4,294,967,295 ms ≈ 49.7 天。
        #    到达最大值后，下一个 tick 不是 +1，而是直接跳回 0（从零开始重新计数）。
        #
        #    🧪 假设就在那个瞬间：
        #       上次 _last_t = 4,294,967,200（溢出前）
        #       本次 tnow    =            100（溢出后，已回绕到小值）
        #       相减  dt      = 100 − 4,294,967,200 = −4,294,967,100  ← 一个巨大的负数！
        #
        #    旧版判断 self._last_t == 0 or dt > 1000：
        #       · _last_t ≠ 0（已经跑过很多帧）→ False
        #       · dt ≈ −43 亿，−43 亿 > 1000 是 False（负数永远不大于正数）
        #       → 整个 if 为 False → 跳过复位 → dt 保持为 −43 亿往下走
        #
        #    后果：
        #       · dt > 0 为 False → D 和 I 全部跳过（仅本帧）
        #       · delta_time = −43 亿 / 1000 = −4,300,000 秒 → 一个荒唐的负数
        #       · 本帧的 self._last_t 被覆盖为 100，下帧 dt 恢复正常
        #       → 只丢一帧的 D/I，不会崩溃，但 delta_time 的坏值是个隐患
        #
        #    修复：加 dt < 0 → 也触反复位，安全度过回绕那一帧。
        # 📎 旧写法：if self._last_t == 0 or dt > 1000:
        if self._last_t == 0 or dt > 1000 or dt < 0:
            dt = 0
            self.reset_I()

        self._last_t = tnow
        delta_time = float(dt) / 1000.0         # 毫秒 → 秒（30fps → ≈0.033s）

        # ── 步骤 2：🟢 P（比例）—— "现在偏了多少" ──
        #   🧮 P = Kp × error，偏得多 → 修正力大。
        #   💡 P 每帧无条件执行，不存在跳过分支，直接赋值一步到位。
        output = error * self._kp

        # ── 步骤 3：🔮 D（微分）—— "变化有多快" ──
        #
        #   📊 数据流：
        #     error ──→ derivative_raw = (error − last_error) / dt   ← 原始速度
        #       │                                                        （含高频噪声）
        #       └──→ EMA 低通滤波 (fc=20Hz) ──→ filtered_derivative
        #                                           │
        #                                           └──→ × Kd ──→ output
        #
        #   🛡️ 摄像头像素抖动 → 虚假高频微分 → D 项乱跳 → 舵机抽搐
        #       EMA 滤波只让 <20Hz 的平滑变化通过，高频毛刺全拦。
        #   🔗 滤波原理详见 __init__._RC
        if self._kd != 0.0 and dt > 0:

            # 🚩 isnan 哨兵：检测微分是否已初始化
            if isnan(self._last_derivative):
                # 🔴 未初始化 — 首次调用 / reset_I() 后，无历史可对比
                derivative = 0.0                # 本帧微分暂给 0（跳过 D）
                self._last_derivative = 0.0     # 切到正常值 → 下帧走 🟢
            else:
                # 🟢 已初始化 — 正常计算原始微分
                derivative = (error - self._last_error) / delta_time   # 🧮 px/s

            # 🧮 EMA 一阶低通：new = old + α × (raw − old)
            #    α = dt / (RC + dt)，RC=1/(2π×20)≈0.008s
            #    30fps 时 α≈0.80 → 尖峰 200 被压到 180 → 连续几帧平滑过渡
            derivative = (self._last_derivative
                          + ((delta_time / (self._RC + delta_time))
                             * (derivative - self._last_derivative)))

            self._last_derivative = derivative    # 📍 保存，供下帧 EMA 用
            output += self._kd * derivative      # ➕ D 项贡献

        # 📍 每帧无条件更新上帧误差。
        #
        # 🐛 旧写法（已修复）：self._last_error = error 在 D 块内部
        #    症状：Kd=0 时 _last_error 永不更新，始终 = 初始值 0
        #    后果：运行中动态启用 D → 首帧 derivative = (error − 0) / dt
        #          → 算出虚假大尖峰 → D 项炸冲 → 车猛甩
        #    修复：移到 D 块外面，每帧无条件执行
        self._last_error = error

        # 🔢 scaler 统一缩放 P + D（I 累加时已乘 scaler，不在此重复）。
        output *= scaler

        # ── 步骤 4：📦 I（积分）—— "偏了多久" ──
        #
        #   🧮 integrator += error × Ki × scaler × dt
        #   🧠 持续小偏差 → P 推不动 → I 逐帧攒大 → 消除稳态残余
        #   🛡️ imax 锁死 [−imax, +imax]，防卡住时积分饱和 → 松手猛冲
        if self._ki != 0.0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            # 🛡️ clamp 到 [−imax, +imax]
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax
            output += self._integrator           # ➕ I 项贡献

        return output


    # ── 1.2 重置积分器 ──────────────────────────

    def reset_I(self):
        '''
        🗑️ 把 PID 的"记忆"全部抹掉，从零开始。

        🧠 打个比方：
           PID 就像一个司机，P=眼睛看现在，I=脑子记过去，D=预判未来。
           reset_I() 就是——把脑子里的旧记忆全部清空，当什么都没发生过。

        🎯 啥时候必须清：
           · 刚启动程序 — 没有"过去"可记
           · 线丢了又找回来 — 丢线前的位置已经是旧黄历了
           · 小车被障碍物卡了半天 — I 已经攒满了，搬开后不能带着满格怒气冲出去
           · 超过 1 秒没调用 — 中间可能停了，时间断档不能接

        ⚠️ 不清会咋样：
           松手后车往旧方向猛冲——因为 I 还记着"往左偏！往左偏！"
        '''
        self._integrator = 0                     # 📦 积分归零 — 忘掉"偏了多久"
        self._last_derivative = float('nan')     # 📍 微分归 nan — 下帧重新初始化
