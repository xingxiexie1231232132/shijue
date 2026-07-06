from gui import GUI
from maix import camera, image, time, app
import math
import cv2
import numpy as np

# ==============================
# 1. 屏幕和控件参数
# ==============================

# 屏幕宽度和高度。
# 这里的 GUI 布局、触摸映射、绘图坐标都按这个分辨率来设计。
_image_width = 320
_image_height = 240

# 两个按钮的尺寸。
_btn_width = _image_width // 6
_btn_height = _image_height // 6

# 按钮 ID。
# 这些 ID 由 GUI 创建后返回，后面用于区分"取阈值"和"二值化"按钮。
_btn_id_pixel = -1
_btn_id_binary = -1

# 当前界面状态。
_to_show_binary = False
_to_get_pixel = False

# ==============================
# 2. 颜色转换：RGB -> LAB
# ==============================

def rgb_to_lab(rgb):
    '''
    将一个 RGB 像素点转换为 LAB 颜色值。

    参数：
        rgb：RGB 三通道颜色，通常格式为 [R, G, B]，范围为 0~255

    返回：
        [L, A, B]：LAB 颜色值，用于后续颜色阈值判断
    '''

    # RGB 到 XYZ 的转换矩阵。
    # 这是颜色空间转换中的中间步骤。
    M = [
        [0.412453, 0.357580, 0.180423],
        [0.212671, 0.715160, 0.072169],
        [0.019334, 0.119193, 0.950227]
    ]

    # 将 RGB 归一化到 0~1。
    r, g, b = rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0

    # 线性化 RGB，去掉伽马校正带来的非线性影响。
    r = r / 12.92 if r <= 0.04045 else ((r + 0.055) / 1.055) ** 2.4
    g = g / 12.92 if g <= 0.04045 else ((g + 0.055) / 1.055) ** 2.4
    b = b / 12.92 if b <= 0.04045 else ((b + 0.055) / 1.055) ** 2.4

    # 计算 XYZ 值。
    X = M[0][0] * r + M[0][1] * g + M[0][2] * b
    Y = M[1][0] * r + M[1][1] * g + M[1][2] * b
    Z = M[2][0] * r + M[2][1] * g + M[2][2] * b

    # XYZ -> LAB 的标准化步骤。
    X /= 0.95047
    Y /= 1.0
    Z /= 1.08883

    def f(t):
        return t ** (1 / 3) if t > 0.008856 else 7.787 * t + 16 / 116

    L = 116 * f(Y) - 16
    a = 500 * (f(X) - f(Y))
    b = 200 * (f(Y) - f(Z))

    return [L, a, b]

# ==============================
# 3. 配置文件读写
# ==============================

def set_configured_threshold(threshold):
    '''
    将当前阈值写入应用配置文件。
    '''

    if len(threshold) < 6:
        return

    app.set_app_config_kv('demo_find_line', 'lmin', str(threshold[0]), False)
    app.set_app_config_kv('demo_find_line', 'lmax', str(threshold[1]), False)
    app.set_app_config_kv('demo_find_line', 'amin', str(threshold[2]), False)
    app.set_app_config_kv('demo_find_line', 'amax', str(threshold[3]), False)
    app.set_app_config_kv('demo_find_line', 'bmin', str(threshold[4]), False)
    app.set_app_config_kv('demo_find_line', 'bmax', str(threshold[5]), True)


def get_configured_threshold():
    '''
    从应用配置文件中读取阈值。
    '''

    threshold = [0, 100, -128, 127, -128, 127]  # 默认阈值

    value_str = app.get_app_config_kv('demo_find_line', 'lmin', '', False)
    if len(value_str) > 0:
        threshold[0] = int(value_str)
    value_str = app.get_app_config_kv('demo_find_line', 'lmax', '', False)
    if len(value_str) > 0:
        threshold[1] = int(value_str)
    value_str = app.get_app_config_kv('demo_find_line', 'amin', '', False)
    if len(value_str) > 0:
        threshold[2] = int(value_str)
    value_str = app.get_app_config_kv('demo_find_line', 'amax', '', False)
    if len(value_str) > 0:
        threshold[3] = int(value_str)
    value_str = app.get_app_config_kv('demo_find_line', 'bmin', '', False)
    if len(value_str) > 0:
        threshold[4] = int(value_str)
    value_str = app.get_app_config_kv('demo_find_line', 'bmax', '', False)
    if len(value_str) > 0:
        threshold[5] = int(value_str)
    return threshold

# ==============================
# 4. 按钮回调
# ==============================

def btn_pressed(btn_id, state):
    '''
    按钮状态变化时触发的回调函数。

    参数：
        btn_id：被操作的按钮 ID
        state：按钮状态，0 表示松开，1 表示按下
    '''

    global _to_show_binary, _to_get_pixel, _btn_id_binary, _btn_id_pixel

    # 这里只响应按键抬起动作，避免按住时重复触发。
    if state == 0:
        return

    if btn_id == _btn_id_binary:
        _to_show_binary = not _to_show_binary
        if _to_get_pixel:
            _to_get_pixel = False
    elif btn_id == _btn_id_pixel:
        _to_get_pixel = not _to_get_pixel



# ==============================
# 5. 快车道三维校验
# ==============================

def check_single_circle(blob, area_threshold):
    '''
    快车道三维联合校验：宽高比 + 充盈率 + 面积。

    三条全部通过 → 认定为干净单圆，直接走思路四（质心法）。
    任一不通过 → 进入慢车道（预处理 + 两段定向扫描）。

    参数：
        blob：find_blobs 返回的色块对象
        area_threshold：最小外接矩形面积阈值

    返回：
        (True, reason)  → 通过快车道
        (False, reason) → 不通过，reason 为失败原因
    '''

    w = blob.w()
    h = blob.h()
    area = w * h
    pixels = blob.pixels()

    # 维度一：面积（噪声拦截）
    if area < area_threshold:
        return False, "面积不足"

    # 维度二：宽高比 → 拦截长条、不规则粘连 ———— 可调参数 ④
    #   正圆外接正方形，w/h ≈ 1.0
    #   允许偏差默认 0.15（15%）：太小→正圆被误拒；太大→椭圆/粘连混入
    ASPECT_MAX_DEV = 0.15
    aspect_deviation = abs(w - h) / max(w, h)
    if aspect_deviation > ASPECT_MAX_DEV:
        return False, "宽高比偏差:{:.2f}".format(aspect_deviation)

    # 维度三：充盈率 → 拦截田字粘连（四个圆宽高比也是 1:1） ———— 可调参数 ⑤
    #   正圆充盈率 = πr² / (2r)² = π/4 ≈ 0.785
    #   下限 0.65：低于此值→内部空洞太多（田字粘连、环形目标）
    #   上限 0.90：高于此值→像素过密，可能是多个碎片合体
    FILL_RATE_MIN = 0.65
    FILL_RATE_MAX = 0.90
    fill_rate = pixels / area
    if fill_rate < FILL_RATE_MIN or fill_rate > FILL_RATE_MAX:
        return False, "充盈率异常:{:.2f}".format(fill_rate)

    return True, "快车道"


# ==============================
# 5b. 边界触边防火墙
# ==============================

def check_edge_clip(blob_rect, margin=3):
    '''
    四向边界校验：检测目标外接矩形是否触碰画面边缘。

    半入镜目标边缘不完整，几何信息失真，
    不应送入快慢车道进行任何拟合计算。

    参数：
        blob_rect：色块外接矩形 [x, y, w, h]
        margin：安全冗余量(px)，默认 3

    返回：
        True  → 触边，需隔离（半入镜残缺目标）
        False → 不触边，正常处理
    '''
    x, y, w, h = blob_rect
    W, H = _image_width, _image_height
    e = margin

    return (x <= e or y <= e or x + w >= W - e or y + h >= H - e)


# ==============================
# 6. 慢车道：距离变换地形分割
# ==============================

def slow_lane_find_centers(img, threshold, blob_rect, pad_ratio=0.15):
    '''
    慢车道多圆心定位：距离变换 + 地形分割。

    核心思想：不找边缘，找重心。
    仅在内部对 ROI 区域做二值化，不存在背景噪点。
    距离变换将平面白块转为三维地形——孤立圆形成单峰，
    粘连花生米形成双峰，中间"脖子"处距离值骤降。
    用水面阈值切割低谷，露出独立山峰，分离粘连体。

    四阶段：
      第一阶段 — ROI 外扩截取 + 局部二值化转 OpenCV
      第二阶段 — 轻开运算去毛刺 → 欧式距离变换生成地形图
      第三阶段 — 水面阈值 0.6×max 切断粘连 → 找轮廓 → 矩求质心
      第四阶段 — 距离场直读内切半径 → 局部坐标映射回全局

    参数：
        img：当前帧 Maix 原始彩色图像
        threshold：LAB 颜色阈值
        blob_rect：色块外接矩形 [x, y, w, h]
        pad_ratio：边框外扩比例，默认 0.15

    返回：
        (centers, radius)
    '''

    x0, y0, w, h = blob_rect

    # ============================================================
    # 第一阶段：绝对净空隔离（仅对 ROI 做二值化）
    #
    # 只在外扩后的 ROI 区域内做二值化，全帧不参与。
    # 背景 = 纯黑 0，目标 = 纯白 255，不存在噪点。
    # ============================================================
    pad = int(max(w, h) * pad_ratio)
    if pad < 3:
        pad = 3

    roi_x0 = max(0, x0 - pad)
    roi_y0 = max(0, y0 - pad)
    roi_x1 = min(_image_width,  x0 + w + pad)
    roi_y1 = min(_image_height, y0 + h + pad)

    # 先裁 ROI（彩色）→ 局部二值化 → 转 OpenCV
    #   只对 ROI 区域做二值化，全帧不参与，算力最小化
    roi_color = img.crop(roi_x0, roi_y0, roi_x1 - roi_x0, roi_y1 - roi_y0)
    roi_binary = roi_color.binary([threshold], False)
    img_cv = image.image2cv(roi_binary, False, False)
    gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # ============================================================
    # 第二阶段：拓扑高地测算
    #
    # 轻开运算刮除边缘极细微毛刺，保持主体平滑。
    # 距离变换计算每一白像素到最近黑背景的欧式距离，
    # 将平面色块转为三维地形——圆心处最高，粘连脖子处骤降。
    # ============================================================

    # 开运算核 ———— 可调参数 ⑦
    #   MORPH_RECT (3,3)：轻量级，仅刮除 <3px 细毛刺
    open_ksize = (3, 3)
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, open_ksize)
    eroded   = cv2.erode(binary, open_kernel, iterations=1)
    cleaned  = cv2.dilate(eroded, open_kernel, iterations=1)

    # 欧式距离变换 ———— 慢车道核心
    #   cv2.DIST_L2：精确欧几里得距离
    #   cv2.DIST_MASK_PRECISE：高精度 5×5 掩码
    #   返回值 dist 是 float32 矩阵，每个元素 = 该像素到最近黑背景的距离
    dist = cv2.distanceTransform(cleaned, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)

    # ============================================================
    # 第三阶段：核心信标提取
    #
    # 取距离场最大值 max_val，以 0.6×max_val 为水面。
    # 淹没低谷（粘连脖子），只露出独立山峰。
    # 山峰互不连通 → findContours 严格对应每个独立小球。
    # ============================================================

    max_val = dist.max()
    if max_val <= 0:
        # 兜底：距离场全零（ROI 中无有效目标）
        return [(x0 + w // 2, y0 + h // 2)], min(w, h) // 2

    # 水面阈值比例 ———— 可调参数 ⑧
    #   0.6：淹没 60% 高度以下区域
    #   ↓ 更低(0.4) → 山峰更大，粘连可能切不断
    #   ↑ 更高(0.8) → 切割更彻底，但可能误切单个扁圆
    WATER_RATIO = 0.6
    water_level = max_val * WATER_RATIO
    _, peaks = cv2.threshold(dist, water_level, 255, cv2.THRESH_BINARY)
    peaks = peaks.astype(np.uint8)

    # 找独立山峰轮廓
    contours, _ = cv2.findContours(
        peaks, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    centers = []

    # 最小山峰面积 ———— 可调参数 ⑨
    #   面积太小说明只是噪声尖峰，非真实球心
    MIN_PEAK_AREA = 5
    # 最小山峰间距 ———— 可调参数 ⑩
    #   两峰中心距离 < N px 视为重复，保留像素面积更大的
    MIN_PEAK_GAP = 5

    for cnt in contours:
        area_peak = cv2.contourArea(cnt)
        if area_peak < MIN_PEAK_AREA:
            continue

        M = cv2.moments(cnt)
        if M['m00'] <= 0:
            continue

        cx_local = M['m10'] / M['m00']
        cy_local = M['m01'] / M['m00']

        # 去重：与已有中心距离 < MIN_PEAK_GAP 视为重复
        duplicate = False
        for ex_cx, ex_cy in centers:
            if abs(cx_local - ex_cx) < MIN_PEAK_GAP and abs(cy_local - ex_cy) < MIN_PEAK_GAP:
                duplicate = True
                break

        if not duplicate:
            centers.append((int(cx_local), int(cy_local)))

    # ============================================================
    # 第四阶段：物理半径反推与全局映射
    #
    # 半径不靠拟合——直接从距离场读取圆心处的距离值。
    # 这个值正是"圆心到最近背景边缘的距离" = 完美内切半径。
    # ============================================================

    # 全局坐标映射 + 距离场直读半径
    global_centers = []
    radius = min(w, h) // 2  # fallback

    for cx_local, cy_local in centers:
        gx = cx_local + roi_x0
        gy = cy_local + roi_y0

        # 边界拦截：ROI 外扩 15% 可能捕获隔壁球的幻影圆心，
        # 若 gx/gy 超出原始 blob_rect 范围则直接丢弃
        if not (x0 <= gx <= x0 + w and y0 <= gy <= y0 + h):
            continue

        global_centers.append((int(gx), int(gy)))

        # 从距离场直读该点的内切半径
        ix = int(cx_local)
        iy = int(cy_local)
        if 0 <= iy < dist.shape[0] and 0 <= ix < dist.shape[1]:
            inscribed_r = dist[iy, ix]
            if inscribed_r > 0:
                radius = int(inscribed_r)

    # 保底：无有效山峰 → 外接矩形中心
    if len(global_centers) < 1:
        global_centers.append((x0 + w // 2, y0 + h // 2))

    return global_centers, radius


# ==============================
# 7. 主程序
# ==============================

def main():
    global _to_show_binary, _to_get_pixel, _btn_id_binary, _btn_id_pixel

    # 打印配置文件路径，方便调试阈值是否保存成功。
    print(app.get_app_config_path())

    # ==================================================================
    # 【全部可调参数一览】
    # ==================================================================
    #
    # ┌─────────────────────┬──────────┬────────────────────────────────┐
    # │ 参数                │ 默认值    │ 含义 / 调参方向                 │
    # ├─────────────────────┼──────────┼────────────────────────────────┤
    # │ _image_width        │ 320      │ 摄像头分辨率宽。↑画质↑算力        │
    # │ _image_height       │ 240      │ 摄像头分辨率高。↑画质↑算力        │
    # ├─────────────────────┼──────────┼────────────────────────────────┤
    # │ _area_threshold     │ 1000     │ 最小外接矩形面积(px²)。↓接受更小  │
    # │                     │          │ 目标，↑过滤更多噪点               │
    # │ _pixels_threshold   │ 1000     │ 最小色块像素数。与面积阈值联动     │
    # ├─────────────────────┼──────────┼────────────────────────────────┤
    # │ [快车道] 宽高比偏差  │ 0.15     │ abs(w-h)/max(w,h)。↓更严 ↑容忍  │
    # │ [快车道] 充盈率下限  │ 0.65     │ pixels/(w*h)。正圆≈0.785         │
    # │ [快车道] 充盈率上限  │ 0.90     │ >0.90 可能是多个粘连碎片          │
    # ├─────────────────────┼──────────┼────────────────────────────────┤
    # │ [慢车道] ROI 外扩比  │ 0.15     │ 边界框外扩比例。↑截取更多边缘     │
    # │ [慢车道] 开运算核    │ (3,3)    │ 刮除毛刺。↑平滑 ↑损失细节         │
    # │ [慢车道] 水面阈值比  │ 0.60     │ 距离场切割水位。↓山峰更大          │
    # │                     │          │ ↑切割更彻底，粘连分离更激进         │
    # │ [慢车道] 最小峰面积  │ 5 px²    │ 孤立山峰的最小面积，过滤噪声峰     │
    # │ [慢车道] 最小峰间距  │ 5 px     │ 两个山峰中心<此值视为重复          │
    # │ [慢车道] 半径来源    │ 距离场   │ inscribed_r = dist[cy, cx]       │
    # │                     │          │ 圆心到最近背景的欧式距离            │
    # ├─────────────────────┼──────────┼────────────────────────────────┤
    # │ [色域] L margin      │ ±50      │ 明度容差。光照变化大时加大        │
    # │ [色域] A margin      │ ±10      │ 红绿色差容差                     │
    # │ [色域] B margin      │ ±10      │ 蓝黄色差容差                     │
    # └─────────────────────┴──────────┴────────────────────────────────┘
    #
    # ==================================================================

    # 摄像头分辨率直接固定为 320 x 240。
    # 这样和 GUI 控件布局更容易对齐。
    cam = camera.Camera(_image_width, _image_height)
    gui = GUI()

    # 左下角按钮：取阈值。
    _btn_id_pixel = gui.createButton(0, _image_height - _btn_height, _btn_width, _btn_height)
    gui.setItemLabel(_btn_id_pixel, '取阈值')
    gui.setItemCallback(_btn_id_pixel, btn_pressed)

    # 右下角按钮：二值化显示。
    _btn_id_binary = gui.createButton(_image_width - _btn_width, _image_height - _btn_height, _btn_width, _btn_height)
    gui.setItemLabel(_btn_id_binary, '二值化')
    gui.setItemCallback(_btn_id_binary, btn_pressed)

    # 色块过滤参数 ———— 可调参数 ①
    #   area_threshold：外接矩形面积过滤。目标在画面中较小时适当降低
    #   pixels_threshold：真正符合颜色阈值的像素数量过滤，与面积阈值联动
    _area_threshold = 1000       # 最小外接矩形面积 (px²)
    _pixels_threshold = 1000     # 最小目标像素数

    # 记录上一次触摸坐标，避免同一个位置重复读取。
    last_x = -1
    last_y = -1

    threshold = get_configured_threshold()
    print(threshold)

    # ================================================================
    # 全局跨帧记忆（迟滞锁定的物理基础） ———— 可调参数 ⑪
    #
    #   上一帧最终胜出者的坐标，-1 表示无目标
    #   TRACKING_RADIUS_SQ：锁定圈半径的平方，省去 sqrt 浮点开方
    # ================================================================
    last_target_cx = -1
    last_target_cy = -1
    TRACKING_RADIUS_SQ = 50 * 50  # 50px 锁定圈

    while not app.need_exit():

        # =========================
        # 1. 获取图像（计算与显示分离）
        #
        # img_raw 永远保持原始彩色数据，所有计算基于它。
        # img_display 用于绘制 + 显示，二值化按钮只影响它。
        # 两者物理隔离，杜绝 UI 状态污染计算数据。
        # =========================
        img_raw = cam.read()
        img_display = img_raw.copy()
        if _to_show_binary:
            img_display = img_display.binary([threshold], False)

        # =========================
        # 2. 点击屏幕取阈值
        # =========================
        if _to_get_pixel:
            x, y = gui.get_touch()
            if last_x != x or last_y != y:
                last_x = x
                last_y = y

                rgb = img_raw.get_pixel(x, y, True)
                lab = rgb_to_lab(rgb)
                if len(lab) >= 3:
                    # ———— 可调参数 ⑥：LAB 阈值自动扩展容差 ————
                    #   以触摸点 LAB 值为中心，向两侧各扩展一定范围
                    #   L(明度) margin=±50：光照波动大时加大到 ±60
                    #   A(红绿) margin=±10：目标颜色纯度高时可减小到 ±5
                    #   B(蓝黄) margin=±10：同上
                    L_MARGIN = 50
                    A_MARGIN = 10
                    B_MARGIN = 10

                    threshold[0] = math.floor(lab[0]) - L_MARGIN
                    threshold[0] = threshold[0] if threshold[0] >= 0 else 0

                    threshold[1] = math.ceil(lab[0]) + L_MARGIN
                    threshold[1] = threshold[1] if threshold[1] <= 100 else 100

                    threshold[2] = math.floor(lab[1]) - A_MARGIN
                    threshold[2] = threshold[2] if threshold[2] >= -128 else -128

                    threshold[3] = math.ceil(lab[1]) + A_MARGIN
                    threshold[3] = threshold[3] if threshold[3] <= 127 else 127

                    threshold[4] = math.floor(lab[2]) - B_MARGIN
                    threshold[4] = threshold[4] if threshold[4] >= -128 else -128

                    threshold[5] = math.ceil(lab[2]) + B_MARGIN
                    threshold[5] = threshold[5] if threshold[5] <= 127 else 127
                    print(threshold)
                    set_configured_threshold(threshold)

            img_display.draw_cross(x, y, image.COLOR_YELLOW, 8, 2)

        # =========================
        # 3. 全目标圆形检测（每目标独立判定快/慢车道）
        # =========================
        blobs = img_raw.find_blobs([threshold], area_threshold=_area_threshold, pixels_threshold=_pixels_threshold)

        if len(blobs) > 0:
            img_display.draw_string(0, 0, "targets:{}".format(len(blobs)), image.COLOR_WHITE)

        # 候选池：本轮所有通过快慢车道的合法圆心
        candidates = []

        for i, blob in enumerate(blobs):
            blob_rect = blob.rect()
            x, y, w, h = blob_rect[0], blob_rect[1], blob_rect[2], blob_rect[3]

            if check_edge_clip(blob_rect):
                img_display.draw_rect(x, y, w, h, image.COLOR_PURPLE, 1)
                img_display.draw_cross(blob.cx(), blob.cy(), image.COLOR_PURPLE, 8, 2)
                img_display.draw_string(x + w + 2, y - 8,
                                        "#{}[EDGE]".format(i), image.COLOR_PURPLE)
                continue

            fast_pass, fast_reason = check_single_circle(blob, _area_threshold)

            if fast_pass:
                circle_x = blob.cx()
                circle_y = blob.cy()
                radius = min(w, h) // 2
                circle_color = image.COLOR_GREEN
                cross_color = image.COLOR_GREEN
                candidates.append({'cx': circle_x, 'cy': circle_y, 'r': radius})
            else:
                centers, radius = slow_lane_find_centers(img_raw, threshold, blob_rect)
                circle_color = image.COLOR_RED
                cross_color = image.COLOR_BLUE

                for ci, (cx, cy) in enumerate(centers):
                    img_display.draw_circle(cx, cy, radius, circle_color, 2)
                    img_display.draw_cross(cx, cy, cross_color, 8, 2)
                    img_display.draw_string(cx + radius + 2, cy + radius + 2,
                                            "C{}".format(ci), circle_color)
                    candidates.append({'cx': cx, 'cy': cy, 'r': radius})

            if fast_pass:
                img_display.draw_circle(circle_x, circle_y, radius, circle_color, 2)
                img_display.draw_cross(circle_x, circle_y, cross_color, 8, 2)
            img_display.draw_rect(x, y, w, h, image.COLOR_WHITE, 1)
            img_display.draw_string(x + w + 2, y - 8, "#{}".format(i), circle_color)

        # ============================================================
        # 5. 候选池结算：权重评分 + 迟滞锁定 + 金色准星
        #
        # 乘区 A（基础面积分）：r×r，省去 π 浮点乘法
        # 乘区 B（空间纵深权重）：底部目标离镜头更近，加权 1.0~1.5
        # 乘区 C（迟滞锁定）：上一帧胜出者附近 ±50px 内 ×1.3
        # ============================================================
        if len(candidates) > 0:
            best = None
            best_score = 0

            for c in candidates:
                cx, cy, r = c['cx'], c['cy'], c['r']

                # 乘区 A：基础面积分
                score = r * r

                # 乘区 B：空间纵深权重
                spatial = 1.0 + (cy / _image_height) * 0.5
                score *= spatial

                # 乘区 C：迟滞锁定
                if last_target_cx >= 0:
                    dx = cx - last_target_cx
                    dy = cy - last_target_cy
                    if dx * dx + dy * dy < TRACKING_RADIUS_SQ:
                        score *= 1.3

                if score > best_score:
                    best_score = score
                    best = c

            # 金色准星覆写：线宽 4、半径 12，完全遮盖原有标识
            img_display.draw_cross(best['cx'], best['cy'],
                                   image.COLOR_YELLOW, 12, 4)

            # 跨帧记忆刷新
            last_target_cx = best['cx']
            last_target_cy = best['cy']
        else:
            # 本帧无候选者 → 重置记忆
            last_target_cx = -1
            last_target_cy = -1

        # =========================
        # 6. 刷新显示
        # =========================
        gui.run(img_display)


if __name__ == '__main__':
    main()
