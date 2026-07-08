from maix import touchscreen, camera, display, image, app
from maix.image import Image
import math


# ============================================================
# GUI 类（内联，避免 MaixPy 单文件运行时找不到模块）
# ============================================================

class GUI:
    def __init__(self) -> None:
        self.background = None
        self.items = list()
        self.callbacks = list()
        self.labels = list()
        self.touch_x = 0
        self.touch_y = 0
        image.load_font("sourcehansans", "/maixapp/share/font/SourceHanSansCN-Regular.otf")
        image.set_default_font("sourcehansans")
        self._ts = touchscreen.TouchScreen()
        self._disp = display.Display()
        self._last_pressed = 0

    def _is_in_item(self, item_id: int, x: int, y: int) -> bool:
        if item_id >= len(self.items) or self.background is None:
            return False
        item_pos = self.items[item_id]
        item_disp_pos = image.resize_map_pos(
            self.background.width(), self.background.height(),
            self._disp.width(), self._disp.height(),
            image.Fit.FIT_CONTAIN,
            item_pos[0], item_pos[1], item_pos[2], item_pos[3]
        )
        return (x > item_disp_pos[0] and x < (item_disp_pos[0] + item_disp_pos[2]) and
                y > item_disp_pos[1] and y < (item_disp_pos[1] + item_disp_pos[3]))

    def createButton(self, x: int, y: int, width: int, height: int) -> int:
        item_id = len(self.items)
        self.items.append([x, y, width, height])
        self.callbacks.append(None)
        self.labels.append(None)
        return item_id

    def setItemCallback(self, item_id: int, cb) -> None:
        if item_id < len(self.items):
            self.callbacks[item_id] = cb

    def setItemLabel(self, item_id: int, label: str) -> None:
        if item_id < len(self.items):
            self.labels[item_id] = label

    def get_touch(self) -> tuple:
        if self.background is None:
            return (0, 0)
        x, y = image.resize_map_pos_reverse(
            self.background.width(), self.background.height(),
            self._disp.width(), self._disp.height(),
            image.Fit.FIT_CONTAIN,
            self.touch_x, self.touch_y
        )
        return (x if x >= 0 else 0, y if y >= 0 else 0)

    def run(self, background: Image) -> None:
        self.background = background
        self.touch_x, self.touch_y, pressed = self._ts.read()
        if self._last_pressed != pressed:
            self._last_pressed = pressed
            for item_id in range(len(self.items)):
                if self._is_in_item(item_id, self.touch_x, self.touch_y):
                    if self.callbacks[item_id] is not None:
                        self.callbacks[item_id](item_id, pressed)
                    break
        for item_id in range(len(self.items)):
            lbl = self.labels[item_id]
            sz = image.string_size(lbl if lbl else "")
            lx = (self.items[item_id][0] + (self.items[item_id][2] - sz.width()) // 2) \
                 if self.items[item_id][2] > sz.width() else self.items[item_id][0]
            ly = (self.items[item_id][1] + (self.items[item_id][3] - sz.height()) // 2) \
                 if self.items[item_id][3] > sz.height() else self.items[item_id][1]
            self.background.draw_rect(
                self.items[item_id][0], self.items[item_id][1],
                self.items[item_id][2], self.items[item_id][3],
                image.COLOR_RED, 2
            )
            if lbl is not None:
                self.background.draw_string(lx, ly, lbl, image.COLOR_WHITE)
        self._disp.show(self.background)

# ============================================================
# 1. 参数配置
# ============================================================

# 颜色阈值 (LAB 颜色空间)
yellow_threshold = [[30, 80, 0, 50, 20, 70]]          # yellow

# 色块过滤参数
area_threshold = 150
pixels_threshold = 100

# ---- 充盈率参数 ----
FILL_RATE_MIN = 0.73         # 低于此值视为粘连体（单球≈0.785）
ASPECT_MAX    = 1.6          # 宽高比超过此值强制进入分离路径（一字形≈3.0）

# ---- 视界线过滤 ----
VIEW_LINE_Y = 60

# ---- 十字交叉投影参数 ----
step_pixels = 10             # 切片步长（像素）

# ---- 迟滞锁定参数 ----
TRACKING_RADIUS = 50
BONUS_MULTIPLIER = 1.3

# ---- 取阈值锚点追踪参数 ----
# 用户取阈值时记录触摸位置为"首选锚点"，后续帧优先跟随该目标
PREFERRED_RADIUS = 80       # 锚点吸引范围（px），建议 >= 球直径
PREFERRED_MULTIPLIER = 3.5  # 锚点加分倍率，需远大于 BONUS_MULTIPLIER


# ============================================================
# 2. 取阈值 / 二值化状态
# ============================================================

_to_show_binary = False
_to_get_pixel   = False
_btn_id_pixel   = -1
_btn_id_binary  = -1


def rgb_to_lab(rgb):
    M = [
        [0.412453, 0.357580, 0.180423],
        [0.212671, 0.715160, 0.072169],
        [0.019334, 0.119193, 0.950227]
    ]
    r, g, b = rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0
    r = r / 12.92 if r <= 0.04045 else ((r + 0.055) / 1.055) ** 2.4
    g = g / 12.92 if g <= 0.04045 else ((g + 0.055) / 1.055) ** 2.4
    b = b / 12.92 if b <= 0.04045 else ((b + 0.055) / 1.055) ** 2.4
    X = M[0][0] * r + M[0][1] * g + M[0][2] * b
    Y = M[1][0] * r + M[1][1] * g + M[1][2] * b
    Z = M[2][0] * r + M[2][1] * g + M[2][2] * b
    X /= 0.95047
    Y /= 1.0
    Z /= 1.08883
    def f(t):
        return t ** (1 / 3) if t > 0.008856 else 7.787 * t + 16 / 116
    L = 116 * f(Y) - 16
    a = 500 * (f(X) - f(Y))
    b = 200 * (f(Y) - f(Z))
    return [L, a, b]


def btn_pressed(btn_id, state):
    global _to_show_binary, _to_get_pixel, _btn_id_binary, _btn_id_pixel
    if state == 0:
        return
    if btn_id == _btn_id_binary:
        _to_show_binary = not _to_show_binary
        if _to_get_pixel:
            _to_get_pixel = False
    elif btn_id == _btn_id_pixel:
        _to_get_pixel = not _to_get_pixel


# ============================================================
# 3. 工具函数
# ============================================================

def calc_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def cross_projection_split(img, threshold, blob_rect, blob_pixels):
    """
    十字交叉投影法：不依赖腐蚀，通过 X/Y 方向切片找多个圆心。

    对所有距离都适用（近/中/远），因为只依赖像素密度分布，
    不依赖色块大小或形态学操作。

    返回：
        [(cx, cy, r), ...] 每个分离出的球的圆心和半径
    """
    x0, y0, w, h = blob_rect
    rough_r = min(w, h) // 2

    # X 轴投影：切竖条
    x_steps = max(w // step_pixels, 2)
    x_data = []
    for i in range(x_steps):
        roi_x = x0 + i * step_pixels
        roi_w = min(step_pixels, x0 + w - roi_x)
        if roi_w <= 0 or h <= 0:
            continue
        bs = img.find_blobs([threshold], roi=[roi_x, y0, roi_w, h],
                            pixels_threshold=3)
        total_px = sum(b.pixels() for b in bs) if bs else 0
        if total_px > 0:
            x_data.append((roi_x + roi_w // 2, total_px))

    # Y 轴投影：切横条
    y_steps = max(h // step_pixels, 2)
    y_data = []
    for j in range(y_steps):
        roi_y = y0 + j * step_pixels
        roi_h = min(step_pixels, y0 + h - roi_y)
        if roi_h <= 0 or w <= 0:
            continue
        bs = img.find_blobs([threshold], roi=[x0, roi_y, w, roi_h],
                            pixels_threshold=3)
        total_px = sum(b.pixels() for b in bs) if bs else 0
        if total_px > 0:
            y_data.append((roi_y + roi_h // 2, total_px))

    # 1D NMS 提取多峰
    def pick_peaks(data, min_gap):
        if not data:
            return []
        sorted_data = sorted(data, key=lambda d: d[1], reverse=True)
        peaks = []
        for coord, px in sorted_data:
            too_close = False
            for p in peaks:
                if abs(coord - p[0]) < min_gap:
                    too_close = True
                    break
            if not too_close:
                peaks.append((coord, px))
        return peaks

    min_gap = max(step_pixels, rough_r // 2)
    x_peaks = pick_peaks(x_data, min_gap)
    y_peaks = pick_peaks(y_data, min_gap)

    if len(x_peaks) < 1:
        x_peaks = [(x0 + w // 2, 0)]
    if len(y_peaks) < 1:
        y_peaks = [(y0 + h // 2, 0)]

    # 交叉验证 + 去重
    centers = []
    for cx, _ in x_peaks:
        for cy, _ in y_peaks:
            rx = max(0, cx - step_pixels // 2)
            ry = max(0, cy - step_pixels // 2)
            check_roi = [rx, ry,
                         min(step_pixels, 320 - rx),
                         min(step_pixels, 240 - ry)]
            bs = img.find_blobs([threshold], roi=check_roi)
            if bs:
                dup = any(abs(cx - e[0]) < step_pixels and abs(cy - e[1]) < step_pixels
                          for e in centers)
                if not dup:
                    centers.append((cx, cy))

    if len(centers) < 1:
        centers = [(x0 + w // 2, y0 + h // 2)]

    # 宽峰补充：若检测到的圆心数少于长轴估算的球数，按主方向等分
    # 一字形时一个方向投影只有一个宽峰，笛卡尔积只产生1个圆心，此处修正
    estimated_n = max(1, round(max(w, h) / max(min(w, h), 1)))
    if estimated_n >= 2 and len(centers) < estimated_n:
        if w >= h:  # 水平一字
            centers = [
                (int(x0 + w * (i + 0.5) / estimated_n), int(y0 + h * 0.5))
                for i in range(estimated_n)
            ]
        else:  # 垂直一字
            centers = [
                (int(x0 + w * 0.5), int(y0 + h * (i + 0.5) / estimated_n))
                for i in range(estimated_n)
            ]

    # 面积守恒半径
    N = len(centers)
    if N > 0 and blob_pixels > 0:
        radius = int(math.sqrt(blob_pixels / (N * math.pi)))
    else:
        radius = rough_r

    return [(cx, cy, radius) for cx, cy in centers]


# ============================================================
# 4. 初始化
# ============================================================

_image_width  = 320
_image_height = 240
_btn_width    = _image_width  // 6
_btn_height   = _image_height // 6

cam = camera.Camera(_image_width, _image_height)
gui = GUI()

_btn_id_pixel  = gui.createButton(0, _image_height - _btn_height, _btn_width, _btn_height)
gui.setItemLabel(_btn_id_pixel, '取阈值')
gui.setItemCallback(_btn_id_pixel, btn_pressed)

_btn_id_binary = gui.createButton(_image_width - _btn_width, _image_height - _btn_height, _btn_width, _btn_height)
gui.setItemLabel(_btn_id_binary, '二值化')
gui.setItemCallback(_btn_id_binary, btn_pressed)

threshold = list(yellow_threshold[0])
last_x, last_y = -1, -1

last_target_cx = -1
last_target_cy = -1

# 用户最近一次取阈值时的触摸坐标（图像坐标系），-1 表示未设置
preferred_cx = -1
preferred_cy = -1


# ============================================================
# 5. 主循环
# ============================================================

while not app.need_exit():

    img_raw     = cam.read()
    img_display = img_raw.copy()
    if _to_show_binary:
        img_display = img_display.binary([threshold], False)

    # ---- 取阈值 ----
    if _to_get_pixel:
        x, y = gui.get_touch()
        if last_x != x or last_y != y:
            last_x, last_y = x, y
            rgb = img_raw.get_pixel(x, y, True)
            lab = rgb_to_lab(rgb)
            if len(lab) >= 3:
                L_MARGIN, A_MARGIN, B_MARGIN = 50, 10, 10
                threshold[0] = max(0,    math.floor(lab[0]) - L_MARGIN)
                threshold[1] = min(100,  math.ceil(lab[0])  + L_MARGIN)
                threshold[2] = max(-128, math.floor(lab[1]) - A_MARGIN)
                threshold[3] = min(127,  math.ceil(lab[1])  + A_MARGIN)
                threshold[4] = max(-128, math.floor(lab[2]) - B_MARGIN)
                threshold[5] = min(127,  math.ceil(lab[2])  + B_MARGIN)
                # 记录触摸位置为首选目标锚点，后续帧优先跟随该球
                preferred_cx = x
                preferred_cy = y
                print("touch({},{}) rgb:{} lab:{:.1f},{:.1f},{:.1f} => {}".format(
                    x, y, rgb, lab[0], lab[1], lab[2], threshold))
        img_display.draw_cross(x, y, image.COLOR_YELLOW, 8, 2)

    blobs = img_raw.find_blobs(
        [threshold],
        area_threshold=area_threshold,
        pixels_threshold=pixels_threshold,
        merge=False
    )

    best_cx = -1
    best_cy = -1
    best_w = 0
    best_h = 0
    max_score = 0

    if blobs:
        for blob in blobs:
            x, y, w, h = blob.rect()
            cx, cy = blob.cx(), blob.cy()
            valid_pixels = blob.pixels()

            # ---- 视界线过滤 ----
            if cy < VIEW_LINE_Y:
                continue

            # ---- 充盈率判断 ----
            bounding_area = w * h
            if bounding_area == 0:
                continue

            fill_rate = valid_pixels / bounding_area
            aspect = max(w, h) / min(w, h) if min(w, h) > 0 else 1.0

            candidates = []  # [(cx, cy, w, h, pixels), ...]

            if fill_rate >= FILL_RATE_MIN and aspect < ASPECT_MAX:
                # 充盈率正常且宽高比接近1 → 单球，直接作为候选
                candidates.append((cx, cy, w, h, valid_pixels))
            else:
                # 充盈率不足或宽高比异常（含一字形） → 粘连体，十字交叉投影分离
                img_display.draw_rect(x, y, w, h, image.COLOR_WHITE, 1)
                balls = cross_projection_split(
                    img_raw, threshold, blob.rect(), valid_pixels
                )
                for bx, by, br in balls:
                    candidates.append((bx, by, br * 2, br * 2, 0))

            # ---- 复合评分 + 迟滞锁定 + 锚点优先 ----
            for cand_cx, cand_cy, cand_w, cand_h, cand_pixels in candidates:
                score_pixels = cand_pixels if cand_pixels > 0 else (cand_w * cand_h)

                base_score = score_pixels
                y_bonus = (cand_cy / 240.0) * base_score * 0.5
                total_score = base_score + y_bonus

                # 普通迟滞：靠近上一帧锁定目标加分
                if last_target_cx != -1 and last_target_cy != -1:
                    dist = calc_distance(cand_cx, cand_cy,
                                         last_target_cx, last_target_cy)
                    if dist < TRACKING_RADIUS:
                        total_score *= BONUS_MULTIPLIER

                # 锚点优先：靠近取阈值时触摸位置加更强分
                # 优先级高于普通迟滞，确保跟随用户指定的球
                if preferred_cx != -1 and preferred_cy != -1:
                    dist_pref = calc_distance(cand_cx, cand_cy,
                                              preferred_cx, preferred_cy)
                    if dist_pref < PREFERRED_RADIUS:
                        total_score *= PREFERRED_MULTIPLIER

                if total_score > max_score:
                    max_score = total_score
                    best_cx = cand_cx
                    best_cy = cand_cy
                    best_w = cand_w
                    best_h = cand_h

    # ---- 绘制结果 ----
    if best_cx >= 0:
        img_display.draw_rect(best_cx - best_w // 2, best_cy - best_h // 2,
                              best_w, best_h, image.COLOR_GREEN, 3)
        img_display.draw_cross(best_cx, best_cy, image.COLOR_GREEN, 10, 2)
        img_display.draw_string(best_cx - best_w // 2,
                                max(0, best_cy - best_h // 2 - 20),
                                "Target", image.COLOR_GREEN)

        last_target_cx = best_cx
        last_target_cy = best_cy
    else:
        last_target_cx = -1
        last_target_cy = -1

    gui.run(img_display)
