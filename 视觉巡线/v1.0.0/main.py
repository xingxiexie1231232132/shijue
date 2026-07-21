'''
test_strip_detection.py (终极自适应抗全盲修复版)
=========================================
功能：MaixCamPro 视觉巡线 —— 视觉调试 + PID 控制量输出版。
     1. 底层 find_blobs 放宽至极限灵敏度，彻底解决镜头抬高时底层全盲不输出的问题。
     2. 上层引入动态像素/宽高比弹性过滤，确保镜头抬高、线变细时系统仍能柔顺追踪。
     3. 修复了上一版全局常量丢失导致的 NameError 崩溃。
     4. 将 12 条带巡线偏差整合为平移/转向误差，并通过 PID 输出控制量。

硬件平台：MaixCamPro (Sipeed)
依赖：    MaixPy >= 4.0, gui.py, pid.py
运行方式：与 gui.py、pid.py 放入同一目录运行。
'''

# === MaixPy 核心库 ===
from maix import camera, image, app, time, uart, pinmap
import math
import struct

from gui import GUI
from pid import PID


# ╔══════════════════════════════════════════╗
# ║  1. 全局常量                              ║
# ╚══════════════════════════════════════════╝

# 📎 旧写法：CAMERA_WIDTH=320, MIDPOINT=160 各自独立赋值
#    问题：改分辨率容易忘改中点，DISP 也重复了相同值
CAMERA_WIDTH   = 320   # QVGA宽度
CAMERA_HEIGHT  = 240
MIDPOINT       = CAMERA_WIDTH // 2   # 画面水平中线，自动跟随宽度
STRIP_COUNT    = 12    # 12条带高密度采样
STRIP_HEIGHT   = 15
START_Y        = 60    # 避开顶部远场杂波
DISP_WIDTH     = CAMERA_WIDTH
DISP_HEIGHT    = CAMERA_HEIGHT
BTN_WIDTH      = DISP_WIDTH // 6
BTN_HEIGHT     = DISP_HEIGHT // 6
STRIP_AREA     = CAMERA_WIDTH * STRIP_HEIGHT   # 单条带面积 4800px，面积上限过滤用

# ── 防误检过滤参数 ──
ADJACENT_STRIP_MAX_DIST = 50   # 容忍抬高后的透视大形变
BLOB_MAX_AREA_RATIO = 0.85

# ── 大弯道宽色块阈值（急弯处色块沿 x 轴拉宽，放大偏差加快转向响应）──
# 📎 旧写法：loop 内直接写 MIDPOINT*0.75 / MIDPOINT*1.75，每次过滤重算
BLOB_WIDE_MIN  = int(MIDPOINT * 0.75)   # 120px — 低于此宽不放大
BLOB_WIDE_MAX  = int(MIDPOINT * 1.75)   # 280px — 高于此宽不放大

# ── 自适应高度与极限细线保障参数 ──
ADAPT_WINDOW       = 30    
ADAPT_INTERVAL     = 10    # 每 10 帧极速更新参数
ADAPT_RATIO_INIT   = 0.4   # 启动或重新取色后的宽高比初值
ADAPT_RATIO_MIN    = 0.35  # 放宽下限，允许抬高后的极细长线通过
ADAPT_RATIO_MAX    = 1.2   
ADAPT_RATIO_MARGIN = 0.65  # 增大安全余量系数
ADAPT_PIXEL_MIN    = 8     # 极限像素下限：低到 8 个像素也抓取
ADAPT_PIXEL_MAX    = 500   # 💡 重新补齐被漏掉的像素上限常量！
ADAPT_PIXEL_INIT   = 20    # 摸底像素阈值
ADAPT_EMA_ALPHA    = 0.4   # 加快滑移跟随响应速度

# ── 丢线恢复参数 ──
RECOVERY_MIN_VALID_POINTS = 2     # 少于 2 个有效点时无法绘制绿线
RECOVERY_TRIGGER_FRAMES   = 3     # 连续 3 帧无有效轨迹后进入恢复模式
RECOVERY_RESET_FRAMES     = 10    # 连续 10 帧丢线后执行一次硬复位
RECOVERY_STABLE_FRAMES    = 3     # 连续找回 3 帧后退出恢复模式
RECOVERY_PIXEL_DECAY      = 0.80  # 恢复中每帧保留 80% 像素门槛
RECOVERY_RATIO_DECAY      = 0.90  # 恢复中每帧保留 90% 宽高比门槛
RECOVERY_RATIO_MIN        = 0.20  # 恢复模式允许 3px/15px 级别的极细线

# ── 底盘运动模式参数 ──
USE_TRANSLATION_PID = True  # True=麦克纳姆轮/全向轮；False=差速轮/舵机车

# ── PID 控制参数 ──
PID_X_KP       = 1.1    # 平移 P：越大越积极修正左右偏移
PID_X_KI       = 0.8    # 平移 I：消除长期贴边误差，过大易累积过冲
PID_X_KD       = 0.01   # 平移 D：抑制横向输出抖动
PID_X_IMAX     = 40     # 平移积分限幅，避免丢线或卡住时积分饱和
PID_TURN_KP    = 1.6    # 转向 P：越大转向越敏感
PID_TURN_KI    = 0.0    # 转向 I：默认不用，避免弯道中长期累积导致甩尾
PID_TURN_KD    = 0.015  # 转向 D：抑制蛇形摆动
PID_SCALER     = 1.0    # PID 输出整体缩放，后续可按下位机协议调整
PID_RESET_FRAMES = 20   # 连续 20 帧无足够轨迹点后清空积分并置零输出


class ProgramState:
    Init    = 0   
    Running = 1   


_context = {
    'state': ProgramState.Init,

    # 📋 threshold — 全局 LAB 阈值模板，取色时一次性写入，格式 [L_min,L_max, A_min,A_max, B_min,B_max]。
    #    🎯 角色：模板（不是直接用于检测的阈值），每次取色后覆盖，各条带从此拷贝。
    'threshold': [0, 100, -128, 127, -128, 127],

    # 📋 thresholds — 12 条带各自的独立阈值副本，运行中各自收紧、互不污染。
    #
    # 🔄 数据流：
    #   threshold（取色写入）
    #     └─→ copy → thresholds[0]  ──→ _tighten_threshold() 收紧
    #              → thresholds[1]  ──→ _tighten_threshold() 收紧
    #              → …                                 ⋮
    #              → thresholds[11] ──→ _tighten_threshold() 收紧
    #
    # ⚠️ 必须用列表推导式 `[... for _ in range(N)]`，不能用 `[[...]] * N`：
    #   后者复制的是同一份列表的引用，收紧一个会污染全部 12 条带。
    'thresholds': [
        [0, 100, -128, 127, -128, 127] for _ in range(STRIP_COUNT)
    ],
    'line_point': [0] * STRIP_COUNT,
    'line_area': [None] * STRIP_COUNT,

    'adapt': {
        # 📐 ratio — ② 宽高比门槛，低于此值视为团状干扰。
        #   初始 ADAPT_RATIO_INIT=0.4，校准后由 _adapt_from_widths() 经 EMA 平滑更新。
        'ratio':     ADAPT_RATIO_INIT,

        # 🔢 pixel_thr — ① 像素数下限，blob[4] < pixel_thr×0.7 则丢弃。
        #   初始 ADAPT_PIXEL_INIT=20，校准后由 _adapt_from_widths() 经 EMA 平滑更新。
        'pixel_thr': ADAPT_PIXEL_INIT,
        # 'area_thr':  ADAPT_PIXEL_INIT,  # 📎 已弃用，始终= pixel_thr，无读取点


        # 📦 线宽样本池 — 元素为 blob[2]（通过四级过滤的色块宽度，单位 px）。
        #
        # 📥 写入：4.4.2 条带通过四级过滤 → append(blob[2])，每帧 ≤12 个
        # 📤 消费：4.4.5 每 10 帧 → 裁剪 → _adapt_from_widths(widths)
        #         │
        #         ├─→ ratio_raw  (w/h × 0.65) → EMA → ratio（宽高比门槛）
        #         └─→ pixel_raw  (w × h × 0.7) → EMA → pixel_thr（像素门槛）
        #         │
        #         └─→ 回到 4.4.2 的 ①② 过滤，闭环自适应
        #
        # 💡 为何保留 360 个而非只留 ~120 个：
        #   ① 实际每帧很少满 12 个（丢线时 0~5），10 帧攒不够中位数；
        #   ② 滑动窗口相邻校准共享 ~240 个，中位数平滑过渡不跳变；
        #   ③ 线宽突变（20→8px）时新旧共存，中位数逐步下移，软着陆。
        #
        # 🗑️ 清空：重新取色 / 进入恢复 / 硬复位

        'widths': [],
        'frame_cnt': 0,             
                  
        # 🎯 校准状态 — 控制 _adapt_from_widths() 的赋值方式：
        #   ❌ False → 直接用粗算值覆盖（旧值失效时 EMA 是累赘）
        #   ✅ True  → EMA 平滑写入（抑制线宽抖动）
        #
        # 🔄 置 False 的三处（旧 EMA 值均不可信）：
        #   · 冷启动 — 程序启动 / 重新取色（line 612），widths 为空。
        #   · 恢复后 — 进入恢复模式（line 922），旧粗线样本已清。
        #   · 硬复位 — 恢复中连丢 10 帧（line 951），门槛打回 ADAPT_PIXEL_MIN。
        #
        # ❗恢复 vs 硬复位
        #   · 恢复：逐帧 ×0.8 渐进降（软着陆，刚好线能过时停住）。
        #   · 硬复位：一步到底 pixel_thr=8（如初始门槛极高，×0.8 降太慢，与其磨不如梭哈）。
        #   首次校准成功 → True，之后新线宽经 EMA 平滑写入。
        'calibrated': False,
        'last_median': 0,

        # 🚩 丢线恢复状态机 — 三个标志位协同工作：
        #
        #   📉 lost_frames   丢线计数器，每帧 valid_count<2 则 +1，≥2 则归零
        #                    达 RECOVERY_TRIGGER_FRAMES=3 → 进入恢复模式
        #
        #   🚩 recovering    恢复模式开关（True=放宽过滤中）
        #                    → 4.4.2 快照到局部变量，跳过 LAB 收紧
        #                    → 4.4.3 写入，下帧生效（延迟 1 帧）
        #
        #   📈 stable_frames 恢复中找回计数器，valid_count≥2 则 +1，否则归零
        #                    达 RECOVERY_STABLE_FRAMES=3 → 退出恢复
        #
        #   状态环：NORMAL ←→ RECOVERY，门槛逐帧放宽 → 找回稳定后退出
        'lost_frames': 0,
        'recovering': False,
        'stable_frames': 0,
    },

    'control': {
        # 🎯 控制量缓存 — 本帧巡线偏差 + PID 输出，供 OSD 显示和下位机读取。
        #   📥 每帧由 _calculate_pid_inputs() 写入 → PID 计算 → 更新 out_x/out_turn
        #   🔄 重新取色 / 丢线超时 / 切回 Init → _reset_pid_control() 全部归零

        # 🎯 最近处有效条带的水平偏差（−160~+160），供平移 PID。
        #   取自底部第一条有效条带；近处最能反映车身横向偏移。
        'move_x': 0.0,

        # 🎯 由下到上折半融合的偏差（−160~+160），供转向 PID。
        #   🧮 move_turn = (move_turn + line_point[n]) / 2，从下往上递推。
        #   📊 底部先进入→被后续反复÷2→权重极小；顶部最后进入→权重 50%。
        #     因此 move_turn 主要"向前看"远处赛道走向，用于转向预判。
        'move_turn': 0.0,

        # 🎯 平移 PID 输出。差速轮/舵机车时固定为 0（无横向平移能力）。
        'out_x': 0.0,

        # 🎯 转向 PID 输出。差速轮：左轮−out_turn / 右轮+out_turn；舵机：90° + out_turn×系数。
        'out_turn': 0.0,

        # 🔢 本帧有效条带数。≥2 正常输出 PID；<2 清零输出、累加 stop_count。
        'valid_count': 0,

        # 📉 连续丢线计数器。≥ PID_RESET_FRAMES(20) 时清空 PID 积分和输出。
        #   💡 延迟 20 帧防误触发——偶发丢线不立即复位，确认"真丢了"才清积分。
        'stop_count': 0,
    },
}


gui = GUI()
pid_x = PID(p=PID_X_KP, i=PID_X_KI, d=PID_X_KD, imax=PID_X_IMAX)
pid_turn = PID(p=PID_TURN_KP, i=PID_TURN_KI, d=PID_TURN_KD)

# ═══════════════════════════════════════════════
#  发送端：UART0 二进制帧 → MSPM0（A16=TX）
#  协议：0xAA + val_x(2B) + val_turn(2B) + checksum(1B)
#  已验证 UART0 默认映射可用，不手动 pinmap
# ═══════════════════════════════════════════════
_uart_ok = False
_uart_err = ""
serial = None

try:
    serial = uart.UART("/dev/ttyS0", 115200)
    _uart_ok = True
except Exception as e:
    _uart_err = str(e)

def send_control_data(out_x, out_turn):
    if not _uart_ok:
        return
    val_x   = int(out_x * 100)
    val_turn = int(out_turn * 100)
    payload = struct.pack('<hh', val_x, val_turn)
    data    = b'\xAA' + payload
    checksum = sum(data) & 0xFF
    packet = data + bytes([checksum])
    serial.write(packet)
    # 每 30 帧打印一次到 MaixVision 终端，不刷屏
    global _send_cnt
    _send_cnt = getattr(send_control_data, '_cnt', 0) + 1
    send_control_data._cnt = _send_cnt
    if _send_cnt % 30 == 1:
        print("[TX #{}] {} | x={:+.2f} turn={:+.2f}".format(
            _send_cnt, packet.hex(), out_x, out_turn))

_btn_continue = -1
_btn_binary   = -1
_btn_lines    = -1
_btn_return   = -1
_btn_pixel    = -1
_to_show_binary = False
_to_show_lines  = False
_to_get_pixel   = False
_prev_get_pixel = False
_last_touch_x   = -1
_last_touch_y   = -1


def _tighten_threshold(threshold):
    '''
    🎯 将单条 LAB 阈值向内收紧一步（步长 5），每次只收紧一个边界。

    调用方用 while 循环反复调用，每次 return True 即完成一步收紧；
    return False 表示全部 6 个边界均已到底，无法继续。

    收紧概念（以 L 通道为例，A/B 同理）：
        L_min ──→                    ←── L_max
        原本范围：[▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒]
        收紧后：    [▒▒▒▒▒▒▒▒▒▒▒]        ← 范围缩小，颜色更精确

    Args:
        threshold: [L_min, L_max, A_min, A_max, B_min, B_max]，原地修改。

    Returns:
        True  → 成功收紧了一个边界（步长 5）
        False → 全部 6 个边界无法再收，通知调用方停止

    收紧优先级（先排除高光，最后收紧暗区）：
        ① L_max −5   高光/白背景干扰最多，优先排除
        ② A_max −5   红色方向上限
        ③ A_min +5   绿色方向下限
        ④ B_max −5   黄色方向上限
        ⑤ B_min +5   蓝色方向下限
        ⑥ L_min +5   阴影里的有效线段，最后再收
    '''
    # ① L_max −5：未达满量程 100，且与 L_min 保持 >5 间距
    if threshold[1] < 100 and threshold[1] > threshold[0] + 5:
        threshold[1] -= 5
        return True

    # ② A_max −5：未达满量程 127，且与 A_min 保持 >5 间距
    if threshold[3] < 127 and threshold[3] > threshold[2] + 5:
        threshold[3] -= 5
        return True

    # ③ A_min +5：未达满量程 −128，且不越过 A_max
    if threshold[2] > -128 and threshold[2] < threshold[3] - 5:
        threshold[2] += 5
        return True

    # ④ B_max −5：未达满量程 127，且与 B_min 保持 >5 间距
    if threshold[5] < 127 and threshold[5] > threshold[4] + 5:
        threshold[5] -= 5
        return True

    # ⑤ B_min +5：未达满量程 −128，且不越过 B_max
    if threshold[4] > -128 and threshold[4] < threshold[5] - 5:
        threshold[4] += 5
        return True

    # ⑥ L_min +5：未达满量程 0，且不越过 L_max
    if threshold[0] > 0 and threshold[0] < threshold[1] - 5:
        threshold[0] += 5
        return True

    # 🛑 全部边界无法再收
    return False


def _adapt_from_widths(widths):
    '''
    根据最近收集到的巡线宽度样本，自适应更新过滤参数。

    这个函数用于解决摄像头高度变化带来的线宽变化：
      - 摄像头离赛道近时，线在画面里更宽，需要更严格的面积/像素过滤；
      - 摄像头抬高后，线在画面里更细，需要降低阈值，避免把有效细线误删。

    Args:
        widths: 最近若干帧中检测到的线宽样本列表，单位为像素。

    Returns:
        None: 直接原地修改 `_context['adapt']` 中的 ratio、pixel_thr、
              last_median 和 calibrated。

    核心公式：
        median_w = widths 的中位数
        ratio_raw = clamp(median_w / STRIP_HEIGHT * ADAPT_RATIO_MARGIN,
                          ADAPT_RATIO_MIN, ADAPT_RATIO_MAX)
        pixel_raw = clamp(int(median_w * STRIP_HEIGHT * 0.7),
                          ADAPT_PIXEL_MIN, ADAPT_PIXEL_MAX)

    参数说明：
        median_w: 当前一批样本的典型线宽，单位为像素。
        STRIP_HEIGHT: 单个巡线条带 ROI 的高度，单位为像素。
        ADAPT_RATIO_MARGIN: 宽高比安全余量，数值越小，形状过滤越宽松。
        ADAPT_RATIO_MIN/MAX: ratio_raw 的安全下限/上限，防止形状阈值过松或过严。
        ADAPT_PIXEL_MIN/MAX: pixel_raw 的安全下限/上限，防止小噪点通过或细线被误删。
        pixel_thr: pixel_raw 经过 EMA 平滑后的动态有效像素数基准，单位为像素个数。
                   它用于判断一个 Blob 内真正符合 LAB 阈值的像素是否足够多，
                   数值越大过滤越严格，数值越小越容易保留远处细线和小色块。
        0.7: 有效填充系数，表示色块不必填满整个“线宽 × 条带高度”的矩形，
             达到约 70% 的估算面积即可作为有效线段。

    EMA 平滑：
        new = old * (1 - ADAPT_EMA_ALPHA) + raw * ADAPT_EMA_ALPHA
        这样参数不会因为某一帧线宽抖动而突然跳变。

    EMA 参数说明：
        old: 当前正在使用的旧参数，如 adapt['ratio'] 或 adapt['pixel_thr']。
        raw: 本轮根据最新线宽直接算出的目标值，如 ratio_raw 或 pixel_raw。
        ADAPT_EMA_ALPHA: 新值权重；值越大响应越快但更容易抖动，
                         值越小越平滑但响应更慢。
        1 - ADAPT_EMA_ALPHA: 旧值权重，用于保留上一轮参数的变化惯性。
    '''
    adapt = _context['adapt']

    # 🚫 样本 < 10 跳过：中位数易被少量异常宽度拉偏
    if len(widths) < 10: return

    # 🧮 中位数：奇数取中间，偶数取中间两数平均
    sorted_w = sorted(widths)
    n = len(sorted_w)
    median_w = sorted_w[n // 2] if n % 2 == 1 else (sorted_w[n // 2 - 1] + sorted_w[n // 2]) / 2.0

    # 📊 median_w → 全部自适应参数的唯一推导源：
    #   median_w ─┬→ last_median     （调试缓存，判断高度变化）
    #             ├→ ratio_raw → EMA → ratio       （② 宽高比门槛）
    #             └→ pixel_raw → EMA → pixel_thr   （① 像素数门槛）

    adapt['last_median'] = median_w

    # 📐 ratio_raw = clamp(median_w / STRIP_HEIGHT × 安全余量, MIN, MAX)
    ratio_raw = max(ADAPT_RATIO_MIN, min(ADAPT_RATIO_MAX, median_w / STRIP_HEIGHT * ADAPT_RATIO_MARGIN))
    # 🔢 pixel_raw = clamp(median_w × STRIP_HEIGHT × 填充系数 0.7, MIN, MAX)
    pixel_raw = max(ADAPT_PIXEL_MIN, min(ADAPT_PIXEL_MAX, int(median_w * STRIP_HEIGHT * 0.7)))

    # ❌ False → 直接覆盖（冷启动）  |  ✅ True → EMA 平滑（抑制抖动）
    #
    # 🧮 EMA 公式（α=ADAPT_EMA_ALPHA=0.4）：
    #   Vₙ = (1−α) × Vₙ₋₁ + α × T  =  0.6 × Vₙ₋₁ + 0.4 × T
    #
    # 📊 新值占比 = 1 − (1−α)ⁿ = 1 − 0.6ⁿ：
    #   n=1→40%, 3→78%, 5→92%, 8→98%
    #
    # 📐 推论：
    #   · 稳态（n→∞）：V = T，EMA 是无偏估计，最终收敛到真实目标值
    #   · α=0.4 平衡点：3 次校准（30 帧≈1s）已吸收 78% 新信息，5 次（50 帧≈1.7s）达 92%
    #   · 对比：α=0.2 需 8 次达 83%——响应慢一倍；α=0.6 虽 2 次达 84%——但抖动明显
    #   · 单帧线宽异常（如 50px 噪点）仅贡献 40% 权重，旧值 60% 压阵，不会剧烈跳变
    #   ⏱️ 约 3~5 次校准即可逼近目标值，几乎无感知延迟。
    if adapt['calibrated']:
        adapt['ratio']     = adapt['ratio'] * (1 - ADAPT_EMA_ALPHA) + ratio_raw * ADAPT_EMA_ALPHA
        adapt['pixel_thr'] = int(adapt['pixel_thr'] * (1 - ADAPT_EMA_ALPHA) + pixel_raw * ADAPT_EMA_ALPHA)
        # adapt['area_thr'] = adapt['pixel_thr']  # 已弃用

    else:
        # ❌ 冷启动 / 恢复后首次：直接用粗算值覆盖，跳过 EMA。
        #    · 冷启动：widths 为空或刚清空（重新取色），旧 EMA 值无意义
        #    · 恢复后：旧 EMA 是粗线场景的（如 pixel_thr=80），新细线仅 8px，
        #      直接覆盖比 EMA 从 80→8 慢降更快，旧值失效时 EMA 是累赘
        adapt['ratio']      = ratio_raw
        adapt['pixel_thr']  = pixel_raw
        # adapt['area_thr'] = pixel_raw  # 已弃用
        adapt['calibrated'] = True   # 🎯 下轮起走 EMA 分支


def draw_cubic_bezier(img, p0, p1, p2, p3, color, steps=15):
    '''
    用 n 段短折线逼近绘制三次贝塞尔曲线。

    三次贝塞尔由 4 个控制点定义：
        p0 ─ 起点，p3 ─ 终点，p1/p2 ─ 中间控制点（控制弯曲方向与程度）。

    算法：将参数 t ∈ [0, 1] 等分为 steps 份，逐点计算伯恩斯坦基函数加权坐标，
    用 draw_line() 连接相邻点。steps 越大曲线越平滑，但绘制开销也越大。

    Args:
        img:    MaixPy Image 对象，绘制目标
        p0~p3:  list[int, int]，四个控制点的 [x, y] 坐标
        color:  image.COLOR_XXX 颜色常量
        steps:  int，分段数，默认 15（在 320×240 画面下足够平滑）

    Returns:
        None: 直接在 img 上绘制，无返回值

    伯恩斯坦基函数（t ∈ [0, 1]）：
        w0 = (1-t)³        起点权重，t=0 时为 1，随 t 增大衰减
        w1 = 3(1-t)²·t     p1 方向拉力，t≈1/3 时权重最大
        w2 = 3(1-t)·t²     p2 方向拉力，t≈2/3 时权重最大
        w3 = t³             终点权重，t=1 时为 1
        w0 + w1 + w2 + w3 ≡ 1，保证曲线始终在控制点凸包内。
    '''
    last_x, last_y = p0[0], p0[1]                   # 从起点开始
    for i in range(1, steps + 1):
        t = i / steps                                # 当前参数 t，步进至 1.0
        temp = 1.0 - t                               # 缓存 (1-t)，减少重复计算
        w0 = temp * temp * temp                      # (1-t)³
        w1 = 3.0 * temp * temp * t                   # 3(1-t)²·t
        w2 = 3.0 * temp * t * t                      # 3(1-t)·t²
        w3 = t * t * t                               # t³

        curr_x = int(w0 * p0[0] + w1 * p1[0] + w2 * p2[0] + w3 * p3[0])
        curr_y = int(w0 * p0[1] + w1 * p1[1] + w2 * p2[1] + w3 * p3[1])

        img.draw_line(last_x, last_y, curr_x, curr_y, color, 2)
        last_x, last_y = curr_x, curr_y              # 本段终点成为下一段起点


def _calculate_pid_inputs(line_points):
    '''
    🎯 将 12 条带偏差转为 PID 的平移误差 (move_x) 和转向误差 (move_turn)。

    🔀 两遍扫描设计：
       第 1 遍 — 从底部向上找第一条有效条带 → 播种 move_x 和 move_turn 初值
       第 2 遍 — 从底部向上融合全部有效条带 → 折半递推得到最终 move_turn
       两遍分开的原因：move_x 只取底部最近一条，move_turn 需要全部融合，
       合并在一个循环里需要额外标志位来区分"初值"和"融合"，拆开更清晰。

    Args:
        line_points: list[int]，长度 12，索引 0=顶部条带 … 11=底部条带。
            每个元素 = center_x − MIDPOINT（−160~+160），
            −500 表示该条带无有效巡线点（哨兵值）。

    Returns:
        tuple[float, float, int]:
            move_x:      最近处有效条带的水平偏差，供平移 PID。
                         ◀ 负=偏左 | 零=居中 | 偏右 ▶ 正
            move_turn:   由下到上折半融合后的偏差，供转向 PID。
                         🧮 递推公式：move_turn ← (move_turn + line_point[n]) / 2
            valid_count: 本帧有效条带数量（0~12）。

    🧠 设计直觉 — 为什么平移和转向"看"的位置不同：
       ┌──────────┬──────────────┬──────────────────────────────────┐
       │          │ 回答的问题    │ 答案在哪里                        │
       ├──────────┼──────────────┼──────────────────────────────────┤
       │ move_x   │ 车现在偏了吗？│ 👇 脚下——底部最近条带             │
       │ (平移)   │              │ 近处偏差直接反映车身横向位置      │
       │          │              │ 若用远处：弯道上车没偏却被推开     │
       ├──────────┼──────────────┼──────────────────────────────────┤
       │ move_turn│ 前方往哪拐？  │ 👆 前方——全部条带融合，远处占优   │
       │ (转向)   │              │ 远处赛道走向决定转弯方向          │
       │          │              │ 若只用近处：到弯道口才拐 → 冲出去 │
       └──────────┴──────────────┴──────────────────────────────────┘

    📊 权重分布（折半递推的数学本质）：
       循环方向 n=11→10→…→0（底部→顶部），每次 ÷2 叠加在旧的累积值上，
       意味着：越早进入 → 被后续 ÷2 的次数越多 → 最终权重越小。

          n=11 (底部,最近)  最先进入 → 被 ÷2 十一次 → 权重 ≈ 0.05%  ← 最小
          n=10             次进入   → 被 ÷2 十次   → 权重 ≈ 0.1%
          ...
          n=1              倒数第二 → 被 ÷2 一次   → 权重 = 25%
          n=0  (顶部,最远)  最后进入 → 被 ÷2 零次   → 权重 = 50%    ← 🏆 最大

       💡 远处占优是刻意的：车头转向应由前方赛道决定（预判入弯），
          近处信号被逐级稀释后只留微弱锚定，不会喧宾夺主。

    🧪 示例（底部→顶部偏差 [−50, −40, −20]，其余无效）：
        move_x = −50（底部最近条带）
        第 2 遍融合：
          n=11: move_turn = (−50 + −50)/2 = −50     ← 种子条带，自我融合不变
          n=10: move_turn = (−50 + −40)/2 = −45
          n=9:  move_turn = (−45 + −20)/2 = −32.5
        展开：−32.5 = −50×25% + −40×25% + −20×50%
        比简单平均 −36.7 明显偏向 −20（顶部远处），因为远处最后进入权重最大。
    '''
    # 🎯 局部变量：move_x 取底部第一条，move_turn 折半融合，valid_count 统计有效条带。
    move_x = 0.0
    move_turn = 0.0
    valid_count = 0

    # 🚩 has_seed — 是否至少找到一条有效条带。
    #   False → 全帧丢线，直接返回 (0, 0, 0)，不送入 PID。
    #   ⚠️ 必须用标志位区分"偏差恰好为 0"和"没有有效条带"——两者 move_x 都是 0.0 但含义完全不同。
    has_seed = False

    # ── 第 1 遍：从底部 (n=11) 向上扫描，找第一条有效条带作为种子 ──
    # 🎯 move_x = 底部第一条 —— "车现在偏了吗？"答案在脚下
    # 🔀 range(STRIP_COUNT-1, -1, -1) = 11, 10, …, 0（底部→顶部）
    #   📥 命中即 break——只取最近处一条，不继续向上。
    for n in range(STRIP_COUNT - 1, -1, -1):
        if line_points[n] > -500:                 # 🛡️ −500 是无效哨兵值，> −500 即有效
            move_x = line_points[n]               # 🎯 平移误差：锁定底部第一条
            move_turn = line_points[n]            # 🔄 转向融合初值：也从同一条开始
            has_seed = True
            break                                 # 🛑 找到即停，不往上扫

    # 🛑 全帧无有效条带 → 不送入 PID，避免积分项吸收无效误差。
    if not has_seed:
        return 0.0, 0.0, 0

    # ── 第 2 遍：从底部向上折半融合，远处权重自动占优 ──
    # 🎯 move_turn = 全部条带加权 —— "前方往哪拐？"答案在前面远处
    # 🧮 move_turn = (move_turn + line_points[n]) / 2.0
    #    · n 越大越先进入 → 被后续 ÷2 次数越多 → 最终权重越小
    #    · 种子条带 (第 1 遍那条) 融合： (x + x)/2 = x → 值不变，可放心重入
    #    · 顶部条带 (n 小) 最后进入 → 权重最大 (n=0 占 50%)
    #    · 无效条带 (> −500 不成立) 直接跳过，不参与融合也不计数
    for n in range(STRIP_COUNT - 1, -1, -1):
        if line_points[n] > -500:
            move_turn = (move_turn + line_points[n]) / 2.0
            valid_count += 1

    return move_x, move_turn, valid_count


def _reset_pid_control():
    '''
    清空 PID 内部记忆和对外输出缓存。

    典型触发场景：
        1. 重新取色后，旧误差对应旧阈值，不应继续影响新场景；
        2. 丢线超过 PID_RESET_FRAMES，积分项可能已经积累无效误差；
        3. 重新进入 Running 阶段时，需要从干净状态开始。
    '''
    control = _context['control']

    pid_x.reset_I()
    pid_turn.reset_I()

    control['move_x'] = 0.0
    control['move_turn'] = 0.0
    control['out_x'] = 0.0
    control['out_turn'] = 0.0
    control['valid_count'] = 0
    control['stop_count'] = 0


def btn_continue_pressed(btn_id, state):
    '''
    "继续"按钮回调：松手时将状态切换为 Running，进入巡线主循环。

    回调签名与触发时机见 CODING_STYLE.md §8.2（state=1 按下时跳过）。
    '''
    global _context
    if state == 1: return
    _context['state'] = ProgramState.Running


def rgb_to_lab(rgb):
    '''
    将单个 RGB 像素转换为 CIE Lab 颜色空间（D65 标准光源）。

    转换管线（sRGB → 线性 RGB → CIE XYZ → CIE Lab）：

        sRGB → 线性 RGB（Gamma 展开）
            用 IEC 61966-2-1 标准的分段函数去 Gamma：
              值 ≤ 0.04045 → 值/12.92（线性段）
              值 > 0.04045 → ((值+0.055)/1.055)^2.4（幂函数段）

        线性 RGB → CIE XYZ（D65 白点）
            标准 sRGB→XYZ 变换矩阵，各行系数为 ITU-R BT.709 原色。

        CIE XYZ → CIE Lab
            先除以 D65 参考白 (xn, yn, zn)，再通过 f(t) 非线性压缩：
              t > 0.008856 → t^(1/3)
              t ≤ 0.008856 → 7.787t + 16/116（线性段，保证低亮区连续）

    Args:
        rgb: list[int]，[R, G, B]，范围 0~255

    Returns:
        list[int]: [L, A, B]
            L:   0~100   （亮度，0=黑 100=白）
            A: -128~127  （绿→红）
            B: -128~127  （蓝→黄）
    '''
    # ── 归一化到 [0, 1] ──
    r, g, b = rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0

    # ── sRGB → 线性 RGB（Gamma 展开）──
    r = r / 12.92 if r <= 0.04045 else ((r + 0.055) / 1.055) ** 2.4
    g = g / 12.92 if g <= 0.04045 else ((g + 0.055) / 1.055) ** 2.4
    b = b / 12.92 if b <= 0.04045 else ((b + 0.055) / 1.055) ** 2.4

    # ── 线性 RGB → CIE XYZ（D65 变换矩阵）──
    x = 0.412453 * r + 0.357580 * g + 0.180423 * b
    y = 0.212671 * r + 0.715160 * g + 0.072169 * b
    z = 0.019334 * r + 0.119193 * g + 0.950227 * b

    # ── XYZ → Lab（D65 参考白 xn=0.95047, yn=1.0, zn=1.08883）──
    xn, yn, zn = 0.95047, 1.0, 1.08883
    x, y, z = x / xn, y / yn, z / zn
    def f(t): return t ** (1.0 / 3) if t > 0.008856 else 7.787 * t + 16.0 / 116

    return [116 * f(y) - 16,            # L：亮度通道
            500 * (f(x) - f(y)),        # A：绿→红
            200 * (f(y) - f(z))]        # B：蓝→黄


def btn_display_mode_pressed(btn_id, state):
    '''
    显示模式切换回调：松手时翻转对应的布尔显示开关。

    由两个按钮共用（btn_binary / btn_lines），通过 btn_id 区分操作目标。
    '''
    global _to_show_binary, _to_show_lines
    if state == 1: return
    if btn_id == _btn_binary: _to_show_binary = not _to_show_binary      # 二值化显隐
    elif btn_id == _btn_lines: _to_show_lines = not _to_show_lines       # 样条线显隐


def btn_return_pressed(btn_id, state):
    '''
    "返回"按钮回调：松手时将状态切回 Init，退出巡线循环回到欢迎页。
    '''
    global _context
    if state == 1: return
    _context['state'] = ProgramState.Init


def btn_pixel_pressed(btn_id, state):
    '''
    触摸取色模式切换回调。

    开启时：关闭二值化显示，将取色光标复位到画面中心。
    关闭时的下降沿由主循环 4.4.1 检测，触发正式 LAB 阈值写入。
    '''
    global _to_get_pixel, _to_show_binary, _last_touch_x, _last_touch_y
    if state == 1: return
    _to_get_pixel = not _to_get_pixel
    if _to_get_pixel:
        _to_show_binary = False                                          # 取色时关闭二值化
        _last_touch_x, _last_touch_y = MIDPOINT, CAMERA_HEIGHT // 2     # 十字光标复位到画面中心

# ╔══════════════════════════════════════════╗
# ║  4. 主程序运行体                          ║
# ╚══════════════════════════════════════════╝

def main():
    '''
    程序入口：创建摄像头，进入 Init ↔ Running 状态机循环。

    状态流转：Init（欢迎页，等待”继续”按钮）→ Running（12 条带巡线 +
    触摸取色）→ Init（”返回”按钮）→ ...，由按钮回调驱动切换。外层
    while not app.need_exit() 保证应用退出时正常结束。
    '''
    # 按钮 ID 需声明 global，才能在回调中写回模块级变量。
    global _btn_continue, _btn_binary, _btn_lines, _btn_return, _btn_pixel, _context

    # 显示/取色状态标志。
    global _to_show_binary, _to_show_lines, _to_get_pixel, _prev_get_pixel

    # 最近一次触摸坐标，用于下降沿读取 RGB 并换算 LAB 阈值。
    global _last_touch_x, _last_touch_y

    # 创建 320×240 摄像头对象。
    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)

    # 🔁 外层状态机循环：
    #   🏠 Init ──点击”继续”──→ 🏃 Running ──点击”返回”──→ 🏠 Init
    #   🚪 app.need_exit() → True 时退出，MaixPy 正常终止。
    while not app.need_exit():

        # ────────────────────────────────────────────────────────────
        # 4.1 Init 欢迎与准备阶段
        # ────────────────────────────────────────────────────────────

        # 切回欢迎页状态。
        _context['state'] = ProgramState.Init
        # GUI 无删除接口，将旧按钮 x 坐标移到屏幕外以隐藏。
        for i in range(len(gui.items)): gui.items[i][0] = -100

        # 右下角创建“继续”按钮（40×20），注册标签和回调。
        _btn_continue = gui.createButton(252, 198, 40, 20)
        gui.setItemLabel(_btn_continue, '继续')
        gui.setItemCallback(_btn_continue, btn_continue_pressed)

        # 欢迎页循环：gui.run() 检测到"继续"按钮松手时，在其内部同步调用
        # btn_continue_pressed() 将 state 改为 Running，本循环随即退出。
        while not app.need_exit() and _context['state'] == ProgramState.Init:
            img = cam.read()

            # 绿色标题栏 + 白色标题文字。
            img.draw_rect(80, 6, 160, 28, image.COLOR_GREEN, 2)
            img.draw_string(98, 10, '视觉巡线调试', image.COLOR_WHITE, 1.2)

            # 操作提示文字。
            img.draw_string(24, 48, '点击 [继续] 开始运行', image.COLOR_YELLOW, 1.0)
            img.draw_string(24, 72, '运行后触摸屏幕取色设定阈值', image.COLOR_WHITE, 1.0)

            # 分隔线 + Running 阶段按钮功能预览。
            img.draw_line(20, 100, DISP_WIDTH - 20, 100, image.COLOR_GREEN, 1)
            img.draw_string(10, 110, '运行中按钮功能:', image.COLOR_GREEN, 1.0)
            img.draw_string(54, 134, '[ 二值化 ]  [ 取阈值 ]  [ 样条线 ]', image.COLOR_WHITE, 0.9)

            # “继续”按钮棕色实心圆底色。
            BTN_BROWN = image.Color.from_rgb(101, 67, 33)
            img.draw_circle(272, 208, 18, BTN_BROWN, -1)

            # 输入、逻辑（含按钮回调）、输出同在这一步内完成。
            # 用户若在本帧松手，回调会在此调用内同步修改 state。
            gui.run(img)

        # 循环退出：非 Running 状态（收到退出请求）→ 结束 main()。
        if _context['state'] != ProgramState.Running: return

        # ────────────────────────────────────────────────────────────
        # 4.2 创建 Running 阶段按钮
        # ────────────────────────────────────────────────────────────
        
        # 将欢迎页“继续”按钮移出屏幕，避免它在运行界面继续响应触摸。
        gui.items[_btn_continue][0] = -100

        # 左下角按钮：切换原图与 LAB 二值化显示。
        _btn_binary = gui.createButton(0, DISP_HEIGHT - BTN_HEIGHT, BTN_WIDTH, BTN_HEIGHT)
        gui.setItemLabel(_btn_binary, '二值化')
        gui.setItemCallback(_btn_binary, btn_display_mode_pressed)

        # 底部居中按钮：开启/关闭触摸取色模式。
        _btn_pixel = gui.createButton(DISP_WIDTH // 2 - BTN_WIDTH // 2, DISP_HEIGHT - BTN_HEIGHT, BTN_WIDTH, BTN_HEIGHT)
        gui.setItemLabel(_btn_pixel, '取阈值')
        gui.setItemCallback(_btn_pixel, btn_pixel_pressed)

        # 右下角按钮：切换色块框和拟合轨迹的叠加显示。
        _btn_lines = gui.createButton(DISP_WIDTH - BTN_WIDTH, DISP_HEIGHT - BTN_HEIGHT, BTN_WIDTH, BTN_HEIGHT)
        gui.setItemLabel(_btn_lines, '样条线')
        gui.setItemCallback(_btn_lines, btn_display_mode_pressed)

        # 右上角按钮：把状态切回 Init，使外层循环重新显示欢迎页。
        _btn_return = gui.createButton(DISP_WIDTH - BTN_WIDTH, 0, BTN_WIDTH, BTN_HEIGHT)
        gui.setItemLabel(_btn_return, '返回')
        gui.setItemCallback(_btn_return, btn_return_pressed)

        # ────────────────────────────────────────────────────────────
        # 4.3 初始化跨帧追踪锚点
        # ────────────────────────────────────────────────────────────

        # 🎯 last_frame_base_x — 帧间锚点，记录上一帧第一个有效点的 x 坐标。
        #
        # ⚠️ 为何需要：本帧循环从 n=0（顶部）开始，但 n=0 没有上方条带可参考，
        #   evaluate_blob() 的距离惩罚 (|cx − tracker_x|) 必须有一个合理的起点。
        #   如果起点偏离车道 → 条带 #0 评分偏低 → 选错色块 → 整帧巡线失败。
        #
        # 🔗 两个锚点的层级关系：
        #   ┌─────────────────────────────────────────────┐
        #   │ last_frame_base_x  ← 帧间锚点（跨帧持久）    │
        #   │   └─→ current_tracker_x ← 帧内锚点（逐带更新）│
        #   │         ├─ n=0: 无上方参考，依赖距离评分选优  │
        #   │         ├─ n>0: 额外叠加 50px 跳变检查        │
        #   │         └─ 有效 → 更新，供下一条带继承         │
        #   └─────────────────────────────────────────────┘
        #
        # 🔄 完整生命周期（跨两帧）：
        #   上帧 4.4.2 — 第一个有效点出现 → last_frame_base_x = center_x（唯一一次写入）
        #      │
        #   ──┼── 帧边界 ──────────────────────────────────
        #      │
        #   本帧 4.4.2 — current_tracker_x = last_frame_base_x（作为 n=0 搜索起点）
        #      │           │
        #      │           ├─ n=0: evaluate_blob() 距离评分 + 跳过跳变检查
        #      │           ├─ n=1: 继承 n=0 的 center_x，叠加 50px 跳变检查
        #      │           ├─ n=2~11: 逐带向下传递 …
        #      │           │
        #      │           └─ len(valid_points)==0 时 → last_frame_base_x = center_x
        #      │                                             （更新，供下帧用）
        #
        # 💡 为何取条带 #0（n=0）的首点作为跨帧锚点：
        #   · 循环从 n=0 开始，第一个有效点自然就是它
        #   · START_Y=60 保证不在顶部噪声区
        #   · 上一帧和本帧的条带 #0 在画面上位置接近（两帧间车道不会瞬移），
        #     用它做参考比用画面中线更准确
        #
        # 🟢 冷启动（首帧）：= MIDPOINT=160，无历史时的合理猜测——车道大概率在画面中间
        # 🔴 本帧更新：= 本帧第一个有效点的 center_x，供下帧用
        # 🛑 重置时机：仅在进入 Running 时执行一次（本段代码），运行中不会重新赋值 MIDPOINT
        last_frame_base_x = MIDPOINT
        _pid_x_disabled_reset = False   # 🚩 差速轮模式下 reset_I() 已执行标记

        _reset_pid_control()

        # ────────────────────────────────────────────────────────────
        # 4.4 Running 逐帧巡线循环
        # ────────────────────────────────────────────────────────────

        # 🔁 Running 循环：state == Running 时持续巡线
        #   🚪 退出：点击“返回” → btn_return_pressed() → state=Init → 回到欢迎页
        while not app.need_exit() and _context['state'] == ProgramState.Running:
            # 每轮读取一帧原始图像，后续取色、色块检测和绘制都基于此帧完成。
            img = cam.read()

            # 🖐️ 4.4.1 触摸取色 — 手指松开时才保存阈值
            #
            # 流程：点"取色"按钮 → 手指在画面上拖动找颜色 → 松开 → 保存
            #
            # 类比拍照：拖动 = 取景（只预览不保存），松开 = 按下快门（正式写入）。
            # 如果拖动时每帧都保存，阈值会跟着手指来回跳，后续检测就乱了。
            #
            # 用 _to_get_pixel（按钮状态）与 _prev_get_pixel（上一帧状态）对比，
            # 精准定位"手指刚松开"的那一帧：
            #
            #   _to_get_pixel  _prev_get_pixel  含义
            #   ❌ False       ❌ False          取色未开启，什么都不做
            #   ✅ True        ❌ False          刚按下按钮，显示十字光标 + LAB 预览
            #   ✅ True        ✅ True           手指拖动中，跟随更新光标位置
            #   ❌ False       ✅ True           🎯 手指刚松开！此时写入最终 LAB 阈值
            if not _to_get_pixel and _prev_get_pixel:
                rgb = img.get_pixel(_last_touch_x, _last_touch_y, True)

                # 🛡️ 防御：img.get_pixel() 偶发返回 None 或残缺列表，
                # 直接传给 rgb_to_lab() 会因 rgb[0] 越界崩溃。
                # 无效时给空元组 () → 下个 if len(lab)>=3 拦截 → 保持旧阈值不变
                lab = rgb_to_lab(rgb) if rgb is not None and len(rgb) >= 3 else ()

                if len(lab) >= 3:
                    # 🎨 以触摸点 LAB 为中心，向两侧扩展构造 [L_min, L_max, A_min, A_max, B_min, B_max]
                    #
                    # 📏 容差（经验值）：
                    #   ☀️ L ±35 — 亮度对阴影/光照敏感，宽范围覆盖不同光线
                    #   🎨 A ±15 — 色度相对稳定，窄范围减少同色背景误检
                    #   🎨 B ±15 — 同上
                    #
                    # 🛡️ clamp 到 LAB 合法范围（L:0~100, A/B:-128~127），防越界
                    th = [0]*6
                    th[0] = max(0,   int(lab[0]) - 35)     # L_min，≥0
                    th[1] = min(100, int(lab[0]) + 35)     # L_max，≤100
                    th[2] = max(-128, int(lab[1]) - 15)    # A_min，≥-128
                    th[3] = min(127,  int(lab[1]) + 15)    # A_max，≤127
                    th[4] = max(-128, int(lab[2]) - 15)    # B_min，≥-128
                    th[5] = min(127,  int(lab[2]) + 15)    # B_max，≤127

                    # 📋 写入全局模板 → 每个条带拷贝一份独立副本，
                    # 后续 _tighten_threshold() 各收各的，互不污染
                    _context['threshold'] = th
                    for n in range(STRIP_COUNT): _context['thresholds'][n] = list(th)

                    # 🔄 新颜色 = 新场景 → 旧线宽样本作废 → 门槛复位 → 重新学习
                    adapt = _context['adapt']
                    adapt['ratio'] = ADAPT_RATIO_INIT
                    adapt['pixel_thr'] = ADAPT_PIXEL_INIT
                    # adapt['area_thr'] = ADAPT_PIXEL_INIT  # 已弃用
                    adapt['widths'].clear()
                    adapt['frame_cnt'] = 0
                    adapt['calibrated'] = False
                    adapt['last_median'] = 0
                    adapt['lost_frames'] = 0
                    adapt['recovering'] = False
                    adapt['stable_frames'] = 0
                    _reset_pid_control()

            # 💡 必须在下降沿判断之后赋值，否则本帧的 _to_get_pixel 会提前覆盖 _prev_get_pixel，导致"刚松开"和"一直没按"无法区分。
            _prev_get_pixel = _to_get_pixel

            # 👆 取色预览 — 手指在画面上拖动时，实时显示十字光标和 LAB 值
            #    ⚠️ 此阶段只预览不保存，松开手指后才通过下降沿触发正式写入
            if _to_get_pixel:
                # 📍 读取触摸坐标
                x, y = gui.get_touch()

                # 🖱️ 光标跟随：坐标变了 → 更新缓存；静止 → 沿用旧坐标
                #    即使手指不动，下方仍每帧重读该点像素（摄像头画面可能变化）
                if x != _last_touch_x or y != _last_touch_y: _last_touch_x, _last_touch_y = x, y

                # ✚ 绘制黄色十字光标（8px 半尺寸，线宽 2，黄色醒目）
                img.draw_cross(_last_touch_x, _last_touch_y, image.COLOR_YELLOW, 8, 2)

                # 🎨 读取触摸点 RGB 像素值
                rgb = img.get_pixel(_last_touch_x, _last_touch_y, True)

                # 🔄 RGB → LAB，在左下角显示实时数值（如 L:50 A:30 B:-10）
                #    🛡️ 读取偶发失败时给空元组 → len<3 → 跳过本帧，不崩溃
                lab = rgb_to_lab(rgb) if rgb is not None and len(rgb) >= 3 else ()
                if len(lab) >= 3: img.draw_string(0, DISP_HEIGHT - 20, 'L:{:.0f} A:{:.0f} B:{:.0f}'.format(lab[0], lab[1], lab[2]), image.COLOR_YELLOW, 0.8)

            # ── 4.4.2 12 条带空间近邻追踪 ──

            # 📎 adapt = _context['adapt'] 的局部别名，省去全路径
            #
            # 🚩 recovering = 恢复模式开关，本帧快照（下帧才生效）
            #
            #   正常模式：blobs>1 → 收紧阈值 → 排除干扰色块 ✅
            #   恢复模式：blobs>1 → 跳过收紧   → 线索碎片不误杀 ✅
            #
            #   原因：恢复模式下远处细线会被 find_blobs() 切成多块碎片，
            #   如果收紧，碎片也会被排除，线就彻底丢了。
            adapt = _context['adapt']
            recovering = adapt['recovering']

            # 📍 valid_points — 轨迹点池，每帧归零，元素为 [center_x, center_y]
            #
            # 🔄 数据流：清零 → 12条带逐条append → 4.4.3 丢线判定 → 4.4.7 贝塞尔拟合
            # 💡 仅记录通过四级过滤的条带，每帧 ≤12 个；与 widths 不同，不跨帧累积
            valid_points = []

            # 🎯 current_tracker_x — 条带间空间追踪锚点
            #
            # 生命周期（每帧）：
            #   起点 ← last_frame_base_x（上帧首点，line 661）
            #     │
            #     ├─→ evaluate_blob() 距离惩罚：|cx − tracker_x|（line 742）
            #     ├─→ ④ 空间连续性过滤：跳变 > 50px 则拒绝（line 810）
            #     │
            #     └─→ 本条带通过四级过滤后更新：tracker_x = center_x（line 820）
            #                    │
            #                    └─→ 下一条带继承此参考点，逐带向下传递
            #
            # 💡 与 last_frame_base_x 的区别：前者跨帧（帧间锚点），后者跨条带（帧内锚点）。
            current_tracker_x = last_frame_base_x

            # n=0~11 自上而下处理 12 个水平条带（ROI y = START_Y + n×STRIP_HEIGHT）。
            for n in range(STRIP_COUNT):
                # 为当前条带复制一份独立阈值，后续收紧不影响其他条带。
                _context['thresholds'][n] = list(_context['threshold'])

                # 旧逻辑（已停用）：
                #   max_iterations, iteration = 5, 0
                #   while iteration < max_iterations:
                #       blobs = img.find_blobs(...)
                #       if blobs and len(blobs) > 1:
                #           if not _tighten_threshold(...): break
                #           iteration += 1
                #       else: break
                # 问题：第 5 次收紧后 iteration 立即达到上限，循环直接结束，
                # blobs 仍是第 4 次收紧后的结果；而且丢线恢复时继续收紧会加剧细线断裂。

                # 新逻辑：普通模式最多收紧 5 次，并在第 5 次后重新检测一次；
                # 恢复模式只用基础 LAB 阈值检测，不执行 _tighten_threshold()。
                max_iterations, iteration = 5, 0
                while True:
                    # ROI: 全宽 320，高 15，y 从 START_Y=60 开始向下排列。
                    # area/pixels 门槛用 ADAPT_PIXEL_MIN=8 保底——先以最低硬下限，捕获远处细线，再交由后续动态过滤清理噪点。
                    blobs = img.find_blobs(
                        thresholds=[_context['thresholds'][n]],
                        roi=[0, START_Y + n * STRIP_HEIGHT, CAMERA_WIDTH, STRIP_HEIGHT],
                        area_threshold=ADAPT_PIXEL_MIN,
                        pixels_threshold=ADAPT_PIXEL_MIN,
                    )

                    # 🛑 四种退出条件（优先级从高到低）：
                    #
                    # 🎯 候选 ≤1 → 阈值已够紧，直接过滤
                    if not blobs or len(blobs) <= 1:
                        break
                    #
                    # 🚩 恢复模式 → 多 Blob 是细线碎片，不收紧
                    if recovering:
                        break
                    #
                    # ⏱️ 已达 5 轮上限 → 不再收紧
                    if iteration >= max_iterations:
                        break
                    #
                    # 📐 收紧一步（步长 5）：成功 → iteration+1 继续；失败 → 已到底
                    if not _tighten_threshold(_context['thresholds'][n]):
                        break

                    iteration += 1

                # ── 当前条带：候选筛选与结果记录 ──
                #
                # 🎯 作用：从 find_blobs() 得到的候选色块中，选出最可靠的一个巡线中心点。
                #
                # 🔍 流程：
                #   候选 blobs
                #     → 按“像素数 + 空间连续性”评分选优
                #     → 依次通过像素下限、宽高比、面积上限、位置连续性过滤
                #     → 通过则记录外接框、偏差、线宽样本和轨迹点
                #     → 失败则标记为无效，并进入下一条带
                #
                # 📌 输出：
                #   line_area[n]      ：有效色块框 [x, y, w, h]
                #   line_point[n]     ：中心相对 MIDPOINT 的水平偏差
                #   adapt['widths']   ：有效线宽样本，用于动态校准
                #   valid_points      ：轨迹点 [center_x, center_y]
                #   current_tracker_x ：当前中心，作为下一条带追踪参考
                #
                # ⚠️ 注意：这里只检查评分最高的候选；
                #   若它被过滤，不会回退尝试第二候选。

                if blobs:
                    # 候选评分：像素越多、距离追踪参考点越近 → 分数越高。
                    # 📎 旧写法：def evaluate_blob(b): … 内嵌函数，每帧重编译 12 次
                    #    改 lambda 一行内联，省去闭包创建开销。
                    blob = max(blobs, key=lambda b: (b[4] * 20) / (abs(b[0] + b[2] // 2 - current_tracker_x) + 1.5))

                    # 📎 旧写法：strip_area = CAMERA_WIDTH * STRIP_HEIGHT 在循环内重算
                    #    改为模块常量 STRIP_AREA = 4800，只算一次
                    
                    # ① 动态像素数下限
                    #
                    # 🧩 pixel_thr：判断线段的有效像素是否足够多。
                    #    比较对象是 blob[4]，即 Blob 内符合 LAB 阈值的像素数。不是外接矩形面积，也不是颜色阈值本身。
                    #
                    # 🔄 来源：初始值为 20；校准时按 
                    #     pixel_raw = median_w * STRIP_HEIGHT * 0.7 估算，再限制到 8~500，并通过 EMA 平滑更新。
                    # ❗【注意】：pixel_thr 在 571 行被动态更新，后续每帧都会重新计算。
                    #
                    # 🚦 门槛：实际要求 blob[4] >= pixel_thr * 0.7，例如：pixel_thr=100 时，低于 70 会被过滤。
                    #
                    # ⚙️ 调参：调大更抗噪但易漏细线；调小更保细线但易进噪点。

                    # 📎 旧写法：_context['adapt']['pixel_thr']，不用局部别名 adapt
                    if blob[4] < adapt['pixel_thr'] * 0.7:
                        _context['line_area'][n], _context['line_point'][n] = None, -500
                        continue

                    # ② 动态宽高比过滤
                    #
                    # 📐 ratio：判断色块形状是否接近“线段”。
                    #   actual_ratio = 宽度 / 高度，数值越大，越像横向线段；
                    #   数值越小，越可能是团状色块或干扰区域。
                    #
                    # 🚦 门槛：当 actual_ratio 低于自适应门槛 ratio，
                    #   且 blob[4] 已大于 pixel_thr 时，说明该色块面积不小，
                    #   但形状不符合线段特征，因此将其过滤。
                    #
                    # ⚙️ 调参：ratio 调大更严格，团状干扰更难通过；
                    #   ratio 调小更宽松，但非线段色块也可能进入后续判断。
                    #   计算候选色块宽高比 w/h；高度为 0 时用 1 兜底，避免除零。
                    actual_h = blob[3] if blob[3] > 0 else 1
                    actual_ratio = blob[2] / actual_h

                    # 📎 旧写法：_context['adapt']['ratio']，不用局部别名 adapt
                    if actual_ratio < adapt['ratio'] and blob[4] > adapt['pixel_thr']:
                        _context['line_area'][n], _context['line_point'][n] = None, -500
                        continue

                    # ③ 面积上限过滤
                    #
                    # 📏 面积：限制 Blob 不能占据条带区域过大。
                    #   当 blob[4] 超过 strip_area * BLOB_MAX_AREA_RATIO 时，通常说明它不是窄线，而是大片阴影、遮挡或同色背景。
                    #
                    # 🚦 门槛：默认按条带面积的 85% 作为上限，超过该比例的色块不参与后续巡线判断。
                    #
                    # ⚙️ 调参：上限调大更宽松，但大面积干扰更容易通过。上限调小更严格，但粗线或近距离线段可能被误过滤。
                    # 📎 旧写法：strip_area（循环内局部变量）= CAMERA_WIDTH * STRIP_HEIGHT
                    #    改为模块常量 STRIP_AREA
                    if blob[4] > STRIP_AREA * BLOB_MAX_AREA_RATIO:
                        _context['line_area'][n], _context['line_point'][n] = None, -500
                        continue

                    # ④ 相邻条带连续性过滤
                    #
                    # 📍 中心点：center_x 表示当前 Blob 的水平中心。
                    #
                    # 🚦 门槛：若 center_x 相对 current_tracker_x 横向跳变超过 ADJACENT_STRIP_MAX_DIST，则过滤。
                    # ❗ 初始值 current_tracker_x = last_frame_base_x = 160（画面中线），因此 n=0 的第一条带没有空间参考，跳过本级过滤。
                    #
                    # ⚙️ 说明：n = 0 没有上方条带参考，跳过本级判断。
                    center_x = blob[0] + blob[2] // 2
                    if abs(center_x - current_tracker_x) > ADJACENT_STRIP_MAX_DIST and n > 0:
                        _context['line_area'][n], _context['line_point'][n] = None, -500
                        continue

                    # ✅ 四级全过 → 📥 线宽入池
                    # 💡 跨帧累积，丢线帧不增也不减；旧样本靠 🗑️ 三个清空点清理
                    # 📎 旧写法：_context['adapt']['widths']，不用局部别名 adapt
                    adapt['widths'].append(blob[2])

                    # 当前有效色块中心成为下一条带的空间参考点。
                    current_tracker_x = center_x

                    # 📦 记录外接框 [x,y,w,h]  +  📏 水平偏差（◀ 左侧 <0 | 0 居中 | >0 右侧 ▶）
                    _context['line_area'][n] = [blob[0], blob[1], blob[2], blob[3]]
                    _context['line_point'][n] = center_x - MIDPOINT

                    # 📍 center_y = START_Y + n×STRIP_HEIGHT + STRIP_HEIGHT//2
                    #    例：n=0→67 … n=11→232 → 📥 存入 valid_points 供贝塞尔拟合
                    #    ⚠️ 坐标不受下方偏差翻倍影响
                    center_y = START_Y + n * STRIP_HEIGHT + STRIP_HEIGHT // 2

                    # 本帧第一个有效点 → 更新下帧搜索锚点 last_frame_base_x。
                    # 💡 若不跟进车道平移，evaluate_blob() 的距离惩罚会导致条带 #0 评分偏低而丢线，连锁引发整帧巡线失败。
                    if len(valid_points) == 0:
                        last_frame_base_x = center_x

                    valid_points.append([center_x, center_y])

                    # 大弯道响应增强：blob 宽度 ∈ (120, 280) 时 line_point ×2
                    # 即 MIDPOINT × 0.75 < blob[2] < MIDPOINT × 1.75
                    #
                    # 急弯处色块沿 x 轴拉宽，放大偏差可加快 PID 转向响应。
                    #
                    # ⚠️ 阴影、粘连、阈值过宽也会产生宽色块，可能误触发。
                    # 范围绑定 320px 画面；更可靠的判断应参考多带曲率。
                    # 本操作仅影响 line_point 输出值，不改变其他变量。
                    # 📎 旧写法：MIDPOINT * 0.75 / MIDPOINT * 1.75，每帧重算
                    if BLOB_WIDE_MIN < blob[2] < BLOB_WIDE_MAX:
                        _context['line_point'][n] *= 2
                else:
                    # 当前条带没有检测到候选色块：清空外接框，并用 -500 表示无效点。
                    _context['line_area'][n], _context['line_point'][n] = None, -500

            # 🔁 4.4.3 丢线恢复状态机
            #
            # ⚠️ 问题：摄像头抬高 / 线变细 → 原 pixel_thr/ratio 误杀细线 → 连续丢线
            #
            # 📊 状态转换：
            #               lost_frames < 3                     lost_frames >= 3
            #   🟢 NORMAL ─────────────────────────────────·──────────────────→ 🔴 RECOVERY
            #     ↑                                                                   │
            #     │  连续 stable_frames >= 3 找回                                     │
            #     └───────────────────────────────────────────────────────────────────┘
            #                               每帧 pixel_thr×0.8, ratio×0.9 渐进放宽
            #
            # 🏷️ 五个标志位：
            #   lost_frames       📉 丢线计数器，≥3 → RECOVERY
            #   stable_frames     📈 找回计数器，≥3 → NORMAL
            #   recovering        🚩 当前是否恢复模式（控制 4.4.2 跳过 LAB 收紧）
            #   calibrated/widths 🗑️ 首次进入恢复时清零，丢弃粗线旧样本
            #
            # 🧪 例：pixel_thr=80，摄像头抬高，线变细被误杀
            #   丢线 3 帧 → 触发 → pixel_thr: 80→64→51→40→32... 逐帧降
            #   找回 3 帧 → 退出 → calibrated=False，下轮用细线粗算值直接覆盖
            #   💡 逐帧降：防噪点瞬间涌入，刚好在"线能通过"时停住
            #
            # ⏱️ 时序：4.4.2 结束 → 无条件进入 → 消费 valid_points 做判定
            #   → 写入 recovering 等 → 下帧 4.4.2 开头快照生效

            # 🐛 旧逻辑（已停用）：丢线时保持门槛不变 → 粗线高门槛锁死
            #   → 细线永远通不过 → 也无法写入 widths → 死循环无法自愈

            valid_count = len(valid_points)
            if valid_count >= RECOVERY_MIN_VALID_POINTS:
                # 至少 2 个点已经能够绘制折线，说明本帧重新获得了有效轨迹。
                adapt['lost_frames'] = 0

                if adapt['recovering']:
                    # 恢复模式不因单帧找回立即退出；连续稳定 3 帧后再恢复正常过滤，避免检测结果在正常/恢复模式之间来回跳变。
                    adapt['stable_frames'] += 1
                    if adapt['stable_frames'] >= RECOVERY_STABLE_FRAMES:
                        adapt['recovering'] = False
                        adapt['stable_frames'] = 0
                        # 💡 退出恢复时不恢复 calibrated——它仍为 False，
                        # 下次 _adapt_from_widths() 走 else 分支，用细线粗算值直接覆盖，
                        # 而非从旧粗线的 EMA 值慢降。
                else:
                    adapt['stable_frames'] = 0
            else:
                # 0~1 个有效点无法形成绿线，累计连续丢线帧数。
                adapt['lost_frames'] += 1
                adapt['stable_frames'] = 0

                if adapt['lost_frames'] >= RECOVERY_TRIGGER_FRAMES:
                    if not adapt['recovering']:
                        # 首次进入恢复模式时立即丢弃近距离旧线宽样本。
                        # 如果继续保留这些粗线样本，后续定期校准可能再次把门槛抬高。
                        adapt['widths'].clear()
                        adapt['frame_cnt'] = 0
                        adapt['calibrated'] = False
                        adapt['last_median'] = 0

                    # 💡 recovering=True 不会跳回 4.4.2——本帧 12 条带已追踪完毕，
                    # 4.4.2 在 4.4.3 之前执行，下帧 4.4.2 开头快照时才读到 True，延迟 1 帧（33ms）对丢线恢复无感知影响。
                    adapt['recovering'] = True

                    # 恢复门槛渐降：
                    #   pixel_thr_new = max(8, pixel_thr_old x 0.80)
                    #   ratio_new     = max(0.20, ratio_old x 0.90)
                    # 每帧逐步放宽可以避免门槛突然降到底后一次放入大量噪点。
                    adapt['pixel_thr'] = max(
                        ADAPT_PIXEL_MIN,
                        int(adapt['pixel_thr'] * RECOVERY_PIXEL_DECAY)
                    )
                    # adapt['area_thr'] = adapt['pixel_thr']  # 已弃用
                    adapt['ratio'] = max(
                        RECOVERY_RATIO_MIN,
                        adapt['ratio'] * RECOVERY_RATIO_DECAY
                    )

                    # 🔨 硬复位 — 恢复模式下连丢 10 帧仍无效，直接打回最低门槛。
                    #
                    # 🆚 与恢复模式的区别：
                    #   · 恢复模式：每帧 ×0.8 渐进降，刚好在"线能过"时停住（软着陆）
                    #   · 硬复位：   一步到位 pixel_thr=8, ratio=0.20（梭哈）
                    #
                    #   例：pixel_thr=500 时丢线，恢复 7 帧后 500×0.8⁷≈105，
                    #   仍远高于 8px 细线，继续磨无意义 → 直接到底，让线必过。
                    #   之后细线通过过滤 → widths 重积 → _adapt_from_widths() 拉回正常值。
                    if adapt['lost_frames'] == RECOVERY_RESET_FRAMES:
                        adapt['pixel_thr'] = ADAPT_PIXEL_MIN
                        # adapt['area_thr'] = ADAPT_PIXEL_MIN  # 已弃用
                        adapt['ratio'] = RECOVERY_RATIO_MIN
                        adapt['widths'].clear()
                        adapt['frame_cnt'] = 0
                        adapt['calibrated'] = False
                        adapt['last_median'] = 0

            # ── 4.4.4 巡线偏差 → PID 控制量 ──
            #
            # 🎯 将 12 条带偏差转为两个 PID 控制量，并处理正常/丢线两路分支。
            #
            # 📊 控制链路全景：
            #
            #   line_point[0..11]（12 条带偏差）
            #     │
            #     └─→ _calculate_pid_inputs()
            #           │
            #           ├─→ move_x      👇 底部第一条 → "车现在偏了吗？"
            #           │      │
            #           │      └─→ pid_x.get_pid(move_x)  ──→ out_x   （横向平移）
            #           │             🔧 Kp=1.1 Ki=0.8 Kd=0.01 Imax=40
            #           │
            #           ├─→ move_turn   👆 12条折半融合 → "前方往哪拐？"
            #           │      │
            #           │      └─→ pid_turn.get_pid(move_turn) ──→ out_turn（车头转向）
            #           │             🔧 Kp=1.6 Ki=0.0 Kd=0.015
            #           │
            #           └─→ pid_valid_count（有效条带数）
            #
            # 🚦 两路分支：
            #   ┌─ pid_valid_count ≥ 2 ─────────────────────────────────────┐
            #   │  ✅ 正常：stop_count = 0，PID 正常输出                     │
            #   │  ⚠️ USE_TRANSLATION_PID=False → out_x 固定为 0           │
            #   │  📤 TODO 处对接串口发送 out_x / out_turn                  │
            #   └──────────────────────────────────────────────────────────┘
            #   ┌─ pid_valid_count < 2 ──────────────────────────────────────┐
            #   │  ❌ 丢线：out_x/out_turn 清零，stop_count + 1              │
            #   │  🛑 stop_count ≥ 20 → _reset_pid_control() 清空积分       │
            #   │  💡 不把不可靠误差送入 PID，避免积分记错方向              │
            #   └──────────────────────────────────────────────────────────┘
            #
            # 📐 符号约定：
            #   line_point = center_x − MIDPOINT（◀ 负=偏左 | 正=偏右 ▶）
            #   若下位机方向相反，在发送前对 out_x / out_turn 取负即可。
            control = _context['control']
            # 📎 旧写法：move_x, move_turn, pid_valid_count = _calculate_pid_inputs(...)
            #    pid_valid_count 在函数内重新遍历 line_point 计数，
            #    但 valid_count = len(valid_points) 已在 4.4.3 算好，完全等价。
            #    改为只取 move_x/move_turn，复用已有的 valid_count。
            move_x, move_turn, _ = _calculate_pid_inputs(_context['line_point'])

            # 📥 写入 control 字典：偏差和计数每帧无条件更新。
            control['move_x'] = move_x
            control['move_turn'] = move_turn
            control['valid_count'] = valid_count

            if valid_count >= RECOVERY_MIN_VALID_POINTS:
                # ✅ 正常模式 — 至少 2 个有效条带，轨迹可信。
                control['stop_count'] = 0                   # 🔄 丢线计数归零

                # 🔀 底盘分叉：平移 PID 仅对全向底盘生效。
                if USE_TRANSLATION_PID:
                    control['out_x'] = pid_x.get_pid(move_x, PID_SCALER)
                else:
                    # ⚠️ 差速轮/舵机车无横向平移自由度，out_x 固定为 0。
                    #    首次进入此行时清一次积分，后续帧 get_pid() 未被调用，
                    #    积分不会累积，无需每帧重复 reset_I()。
                    # 📎 旧写法：每帧都调 pid_x.reset_I()，浪费 CPU + 分配 float('nan')
                    control['out_x'] = 0.0
                    if not _pid_x_disabled_reset:
                        pid_x.reset_I()
                        _pid_x_disabled_reset = True
                if USE_TRANSLATION_PID:
                    _pid_x_disabled_reset = False

                control['out_turn'] = pid_turn.get_pid(move_turn, PID_SCALER)

                send_control_data(control['out_x'], control['out_turn'])
            else:
                # ❌ 丢线模式 — 有效条带 < 2，轨迹不可信。
                #   💡 不送入 PID：误差是随机噪声，积分记错方向后需要很长时间纠正。
                control['stop_count'] += 1                  # 📉 连续丢线累加
                control['out_x'] = 0.0                      # 🛑 输出清零
                control['out_turn'] = 0.0

                # 🛑 连续丢线 20 帧 → 彻底清空 PID 积分和输出缓存。
                if control['stop_count'] >= PID_RESET_FRAMES:
                    _reset_pid_control()

                send_control_data(0.0, 0.0)

            # ── 4.4.5 线宽自适应校准（每 10 帧一次）──
            #
            # 检测与校准不同频：每帧收集线宽到 widths，每 ADAPT_INTERVAL=10 帧才调用 _adapt_from_widths() 重算 pixel_thr / ratio 。
            #
            # 样本池最多保留 ADAPT_WINDOW × STRIP_COUNT = 360 个宽度，超出取最新；
            # 少于 10 个时 _adapt_from_widths() 内部直接跳过，不参与中位数/EMA 计算。
            # 校准数据可跨周期复用，参数平滑过渡；长期无新样本则保留旧场景特征。
            adapt['frame_cnt'] += 1
            if adapt['frame_cnt'] >= ADAPT_INTERVAL:
                adapt['frame_cnt'] = 0

                # 裁剪旧样本，只保留末尾最新 360 个宽度
                if len(adapt['widths']) > ADAPT_WINDOW * STRIP_COUNT:
                    adapt['widths'] = adapt['widths'][-(ADAPT_WINDOW * STRIP_COUNT):]

                # 中位数 → ratio_raw / pixel_raw → EMA 平滑写入（详见函数 docstring）
                _adapt_from_widths(adapt['widths'])

            # ── 4.4.6 选择原图或二值化显示画布 ──

            if _to_show_binary:
                # copy() 生成副本，避免 binary() 改变后续仍需使用的原始摄像头帧。
                # 采用最后一个条带的收紧后 LAB 阈值进行二值化；False 表示不反转结果。
                disp_img = img.copy().binary([_context['thresholds'][STRIP_COUNT - 1]], False)
            else:
                # 未开启二值化时，直接在原始图像上叠加检测结果和调试信息。
                disp_img = img

            # ── 4.4.7 绘制色块框与巡线轨迹 ──

            if _to_show_lines:
                # 蓝色矩形表示每个条带中通过全部过滤的有效线段外接框。
                for n in range(STRIP_COUNT):
                    if _context['line_area'][n] is not None:
                        box = _context['line_area'][n]
                        disp_img.draw_rect(box[0], box[1], box[2], box[3], image.COLOR_BLUE, 1)

                # 根据本帧有效采样点数量选择轨迹绘制策略。
                num_pts = len(valid_points)
                if num_pts >= 4:
                    # 至少 4 个点时，从首点、1/3 点、2/3 点和末点抽取控制点，
                    # 构成三次贝塞尔曲线；steps=15 表示用 15 段短线近似平滑曲线。
                    p0 = valid_points[0]                
                    p1 = valid_points[num_pts // 3]       
                    p2 = valid_points[2 * num_pts // 3]   
                    p3 = valid_points[-1]               
                    draw_cubic_bezier(disp_img, p0, p1, p2, p3, image.COLOR_GREEN, steps=15)
                elif 1 < num_pts < 4:
                    # 只有 2~3 个点时不足以稳定构造四控制点贝塞尔曲线，
                    # 因此退化为逐点绿色折线，避免轨迹完全不显示。
                    for idx in range(num_pts - 1):
                        disp_img.draw_line(valid_points[idx][0], valid_points[idx][1],
                                           valid_points[idx+1][0], valid_points[idx+1][1],
                                           image.COLOR_GREEN, 2)

            # ── 4.4.8 叠加 OSD 调试信息 ──

            # 左上角显示当前基础 LAB 阈值：[L_min,L_max]、[A_min,A_max]、[B_min,B_max]。
            th = _context['threshold']
            disp_img.draw_string(0, 0, 'L:[{:d},{:d}] A:[{:d},{:d}] B:[{:d},{:d}]'.format(th[0], th[1], th[2], th[3], th[4], th[5]), image.COLOR_YELLOW, 0.6)

            # last_median 尚无有效值时显示“--”，否则显示最近一次线宽中位数（像素）。
            median_str = '{:.0f}'.format(adapt['last_median']) if adapt['last_median'] > 0 else '--'

            # 底部状态格式：R=动态宽高比门槛，Px=动态像素门槛，W=中位线宽。
            # [V] 已校准；[~] 正在采样；[R数字] 正在恢复，数字为连续丢线帧数。
            mode_str = 'R{}'.format(adapt['lost_frames']) if adapt['recovering'] else ('V' if adapt['calibrated'] else '~')
            disp_img.draw_string(0, 220, 'R:{:.2f} Px:{} W:{} [{}]'.format(adapt['ratio'], adapt['pixel_thr'], median_str, mode_str),
                                 image.COLOR_YELLOW if adapt['recovering'] or not adapt['calibrated'] else image.COLOR_WHITE, 0.6)

            # 逐行显示 n0~n11 的水平偏差：有效值用绿色，-500 无效标记显示为“--”并用红色。
            for n in range(STRIP_COUNT):
                pt = _context['line_point'][n]
                disp_img.draw_string(0, 12 + n * 10, 'n{}:{}'.format(n, '{:4d}'.format(pt) if pt > -500 else '  --'), image.COLOR_GREEN if pt > -500 else image.COLOR_RED, 0.7)

            # PID 调试信息：
            #   Mx/Mt 为送入 PID 的平移/转向误差；Ox/Ot 为 PID 输出。
            #   V 表示本帧有效条带数，S 表示连续 PID 无效帧数。
            control = _context['control']
            pid_color = image.COLOR_ORANGE if control['valid_count'] >= RECOVERY_MIN_VALID_POINTS else image.COLOR_YELLOW
            x_pid_state = 'ON' if USE_TRANSLATION_PID else 'OFF'
            disp_img.draw_string(0, 136, 'PID V:{}/{} S:{} X:{}'.format(control['valid_count'], STRIP_COUNT, control['stop_count'], x_pid_state), pid_color, 0.6)
            disp_img.draw_string(0, 148, 'Mx:{:.0f} Mt:{:.0f}'.format(control['move_x'], control['move_turn']), pid_color, 0.6)
            disp_img.draw_string(0, 160, 'Ox:{:.1f} Ot:{:.1f}'.format(control['out_x'], control['out_turn']), pid_color, 0.6)
            uart_color = image.COLOR_GREEN if _uart_ok else image.COLOR_RED
            uart_msg   = 'TX:OK' if _uart_ok else ('TX:ERR ' + _uart_err[-8:])
            disp_img.draw_string(0, 172, uart_msg, uart_color, 0.6)

            # ── 4.4.9 绘制 Running 阶段按钮底色 ──

            # GUI 类只负责按钮标签和触摸区域，因此这里先绘制四个棕色实心圆作为按钮底色：
            # 左下=二值化，中下=取阈值，右下=样条线，右上=返回。
            BTN_BROWN = image.Color.from_rgb(101, 67, 33)
            disp_img.draw_circle(BTN_WIDTH // 2, DISP_HEIGHT - BTN_HEIGHT // 2, 20, BTN_BROWN, -1)
            disp_img.draw_circle(DISP_WIDTH // 2, DISP_HEIGHT - BTN_HEIGHT // 2, 20, BTN_BROWN, -1)
            disp_img.draw_circle(DISP_WIDTH - BTN_WIDTH // 2, DISP_HEIGHT - BTN_HEIGHT // 2, 20, BTN_BROWN, -1)
            disp_img.draw_circle(DISP_WIDTH - BTN_WIDTH // 2, BTN_HEIGHT // 2, 20, BTN_BROWN, -1)

            # ── 4.4.10 处理触摸并刷新显示 ──

            # gui.run() 读取触摸状态、执行命中的按钮回调、绘制按钮文字，
            # 最后调用显示屏接口输出本帧完整画面。
            gui.run(disp_img)


if __name__ == '__main__':
    try: main()
    except Exception as e: print("Program exception:", e)
