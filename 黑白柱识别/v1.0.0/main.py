# main.py — 巡线（线性回归）双色检测版
from gui import GUI
from maix import camera, image, time, app
import math

# ==========================================================
# 全局配置
# ==========================================================
_image_width  = 320
_image_height = 240
_btn_width    = _image_width // 6
_btn_height   = _image_height // 6
_center_x     = _image_width // 2

_btn_id_pixel  = -1
_btn_id_binary = -1
_btn_id_focus  = -1

_to_show_binary = False
_to_get_pixel   = False
_focus_mode     = False
_gui_ref        = None

# ==========================================================
# 3×3 地图 + 车位状态
# ==========================================================
# 默认地图: 0=未知, 1=白柱, 2=黑柱
_world_map = [
    [2, 1, 1],
    [2, 2, 2],
    [1, 2, 1]
]
# 车位: row 0-3 (0=出口C), col 0-3 (0=左墙 1=中左 2=中右 3=右墙), heading 0=N 1=E 2=S 3=W
_car_state = {'row': 3, 'col': 2, 'heading': 0}
_map_mode   = False

# 地图模式下新增的按钮 ID
_btn_id_turn_left  = -1
_btn_id_turn_right = -1
_correct_locked_until = 0  # 移动后 1s 内禁止自动修正
_pillar_locked = set()     # 已确认为黑柱的 (row, col)，永久锁定不再修改

# ==========================================================
# RGB → LAB
# ==========================================================
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
    X = M[0][0]*r + M[0][1]*g + M[0][2]*b
    Y = M[1][0]*r + M[1][1]*g + M[1][2]*b
    Z = M[2][0]*r + M[2][1]*g + M[2][2]*b
    X /= 0.95047; Y /= 1.0; Z /= 1.08883
    def f(t): return t**(1/3) if t > 0.008856 else 7.787*t + 16/116
    L = 116*f(Y) - 16
    a = 500*(f(X) - f(Y))
    b = 200*(f(Y) - f(Z))
    return [L, a, b]

def mk_threshold(lab, L_margin=30, A_margin=15, B_margin=15):
    """以 LAB 值为中心生成阈值区间。
    白色大幅收紧 L 容差并抬高下限，灰色赛道（L 35~55）绝不被误认。"""
    L = lab[0]
    if L >= 60:
        L_margin = 12
        A_margin = 6
        B_margin = 6
    elif L > 40:
        L_margin = 8
    th = [0]*6
    lo = max(0,   math.floor(L) - L_margin)
    hi = min(100, math.ceil(L)  + L_margin)
    th[0] = lo
    th[1] = hi
    th[2] = max(-128, math.floor(lab[1]) - A_margin)
    th[3] = min(127,  math.ceil(lab[1])  + A_margin)
    th[4] = max(-128, math.floor(lab[2]) - B_margin)
    th[5] = min(127,  math.ceil(lab[2])  + B_margin)
    return th

# ==========================================================
# 车位移动 & 按钮标签同步
# ==========================================================
def _car_forward():
    """朝当前 heading 方向前进一步（不越界）"""
    r, c, h = _car_state['row'], _car_state['col'], _car_state['heading']
    if h == 0 and r > 0:
        _car_state['row'] = r - 1
    elif h == 1 and c < 3:
        _car_state['col'] = c + 1
    elif h == 2 and r < 3:
        _car_state['row'] = r + 1
    elif h == 3 and c > 0:
        _car_state['col'] = c - 1

def _sync_button_labels():
    """根据当前模式同步按钮标签"""
    if _gui_ref is None:
        return
    if _map_mode:
        _gui_ref.setItemLabel(_btn_id_pixel, '前进')
        _gui_ref.setItemLabel(_btn_id_turn_left, '左转')
        _gui_ref.setItemLabel(_btn_id_turn_right, '右转')
        _gui_ref.setItemLabel(_btn_id_binary, '')
        _gui_ref.setItemLabel(_btn_id_focus, '[地图]')
        _gui_ref.setItemActive(_btn_id_focus, True)
    else:
        _gui_ref.setItemLabel(_btn_id_pixel, '取阈值')
        _gui_ref.setItemLabel(_btn_id_turn_left, '')
        _gui_ref.setItemLabel(_btn_id_turn_right, '')
        _gui_ref.setItemLabel(_btn_id_binary, '二值化')
        _gui_ref.setItemLabel(_btn_id_focus, '[ + ]')
        _gui_ref.setItemActive(_btn_id_focus, False)

# ==========================================================
# 配置文件读写
# ==========================================================
def set_configured_thresholds(t1, t2):
    for i, v in enumerate(t1):
        app.set_app_config_kv('line_find', 't1_'+str(i), str(v), False)
    for i, v in enumerate(t2):
        app.set_app_config_kv('line_find', 't2_'+str(i), str(v), True)

def get_configured_thresholds():
    t1 = [0, 100, -128, 127, -128, 127]
    t2 = [0, 100, -128, 127, -128, 127]
    for i in range(6):
        v = app.get_app_config_kv('line_find', 't1_'+str(i), '', False)
        if len(v) > 0: t1[i] = int(v)
        v = app.get_app_config_kv('line_find', 't2_'+str(i), '', False)
        if len(v) > 0: t2[i] = int(v)
    return t1, t2

# ==========================================================
# 按钮回调与防抖
# ==========================================================
_last_btn_time = 0

def btn_pressed(btn_id, state):
    global _to_show_binary, _to_get_pixel, _focus_mode, _map_mode
    global _gui_ref, _last_btn_time, _correct_locked_until
    if state == 0: return

    now = time.ticks_ms()
    if now - _last_btn_time < 300:
        return
    _last_btn_time = now

    # 地图模式下的按钮
    if _map_mode:
        if btn_id == _btn_id_focus:
            _map_mode = False
            _sync_button_labels()
        elif btn_id == _btn_id_pixel:
            _car_forward()
            _correct_locked_until = time.ticks_ms() + 1000
        elif btn_id == _btn_id_turn_left:
            _car_state['heading'] = (_car_state['heading'] - 1) % 4
            _correct_locked_until = time.ticks_ms() + 1000
        elif btn_id == _btn_id_turn_right:
            _car_state['heading'] = (_car_state['heading'] + 1) % 4
            _correct_locked_until = time.ticks_ms() + 1000
        return

    # 正常模式
    if btn_id == _btn_id_binary:
        _to_show_binary = not _to_show_binary
    elif btn_id == _btn_id_pixel:
        _to_get_pixel = not _to_get_pixel
    elif btn_id == _btn_id_focus:
        _map_mode = True
        _sync_button_labels()

# 未配置的默认全匹配阈值（全色域，会导致二值化全屏命中）
_DEFAULT_TH = [0, 100, -128, 127, -128, 127]

def _is_valid_th(th):
    """阈值不等于全匹配默认值，说明已被取色配置过"""
    return th != _DEFAULT_TH

# ==========================================================
# 形态过滤：柱体长宽比 + 面积
# ==========================================================
def is_pillar(blob):
    """长条形柱体：必须直立，排除上1/3屏、横条和巨型背景"""
    w, h = blob.w(), blob.h()
    if w < 3 or h < 3 or blob.pixels() < 40:
        return False
    # 必须垂直：高 > 宽 × 1.5，排除倒地柱和横向色块
    if h < w * 1.5:
        return False
    # 宽度上限：超过 80px 为白墙/地板
    if w > 80:
        return False
    # 上 1/3 屏屏蔽：色块完全在上 80px 内 → 不认
    if blob.y() + blob.h() <= 80:
        return False
    ratio = max(w, h) / min(w, h)
    return ratio >= 1.8

# ==========================================================
# 地图绘制
# ==========================================================
_MAP_LEFT = 90
_MAP_TOP  = 50
_MAP_CELL = 35

def draw_map(img):
    """在图像上绘制 4×4 围墙网格 + 3×3 柱子 + 车位"""
    # 外框（围墙）
    img.draw_rect(_MAP_LEFT, _MAP_TOP, _MAP_CELL * 4, _MAP_CELL * 4,
                  image.Color.from_rgb(255, 255, 255), 3)
    # 内部网格线
    for i in range(1, 4):
        x = _MAP_LEFT + i * _MAP_CELL
        y = _MAP_TOP + i * _MAP_CELL
        img.draw_line(x, _MAP_TOP, x, _MAP_TOP + _MAP_CELL * 4,
                      image.Color.from_rgb(128, 128, 128), 1)
        img.draw_line(_MAP_LEFT, y, _MAP_LEFT + _MAP_CELL * 4, y,
                      image.Color.from_rgb(128, 128, 128), 1)
    # 9 个柱子（3×3 交点）
    for pr in range(3):
        for pc in range(3):
            px = _MAP_LEFT + (pc + 1) * _MAP_CELL
            py = _MAP_TOP + (pr + 1) * _MAP_CELL
            c = _world_map[pr][pc]
            if c == 1:
                img.draw_circle(px, py, 7, image.COLOR_WHITE, -1)
                img.draw_circle(px, py, 7, image.COLOR_BLACK, 1)
            elif c == 2:
                img.draw_circle(px, py, 7, image.COLOR_BLACK, -1)
            else:
                img.draw_circle(px, py, 7, image.Color.from_rgb(128, 128, 128), -1)
    # 车位（绿色圆 + 红色朝向线）
    r, c, h = _car_state['row'], _car_state['col'], _car_state['heading']
    cx = _MAP_LEFT + c * _MAP_CELL + _MAP_CELL // 2
    cy = _MAP_TOP + r * _MAP_CELL + _MAP_CELL // 2
    dx = {0: 0, 1: 8, 2: 0, 3: -8}.get(h, 0)
    dy = {0: -8, 1: 0, 2: 8, 3: 0}.get(h, 0)
    img.draw_circle(int(cx), int(cy), 6, image.COLOR_GREEN, -1)
    img.draw_line(int(cx), int(cy), int(cx + dx), int(cy + dy),
                  image.COLOR_RED, 2)
    # 起止标注
    img.draw_string(_MAP_LEFT + _MAP_CELL * 2 + 5, _MAP_TOP + _MAP_CELL * 3 + 5,
                    "A", image.Color.from_rgb(255, 255, 0))
    img.draw_string(_MAP_LEFT + _MAP_CELL + 5, _MAP_TOP - 15,
                    "C", image.Color.from_rgb(255, 255, 0))
    # 车位信息
    dir_name = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}.get(h, '?')
    img.draw_string(_MAP_LEFT, _MAP_TOP + _MAP_CELL * 4 + 8,
                    'R{}C{} {}'.format(r, c, dir_name),
                    image.Color.from_rgb(200, 200, 200))

# ==========================================================
# 自动地图修正
# ==========================================================
def auto_correct_map(is_left, is_t1=None):
    """根据当前车位修正世界地图。
    is_t1=None → 无柱子，设为白色(1)
    is_t1=True → 取色1(黑柱)，设为黑色(2)
    is_t1=False → 取色2(白柱)，设为白色(1)
    移动后 1s 内禁止修正。"""
    global _correct_locked_until
    if time.ticks_ms() < _correct_locked_until:
        return
    r, c, h = _car_state['row'], _car_state['col'], _car_state['heading']
    if h != 0:
        return
    target_row = r - 1
    target_col = c - 1 if is_left else c
    if 0 <= target_row < 3 and 0 <= target_col < 3:
        if (target_row, target_col) in _pillar_locked:
            return  # 该柱子已锁定为黑柱，永久不可修改
        if is_t1 is True:
            new_val = 2
            _pillar_locked.add((target_row, target_col))  # 确认黑柱后锁定
        else:
            new_val = 1
        if _world_map[target_row][target_col] != new_val:
            print('地图修正: [{},{}] {}→{}'.format(target_row, target_col,
                                                   _world_map[target_row][target_col], new_val))
            _world_map[target_row][target_col] = new_val

# ==========================================================
# 主程序
# ==========================================================
def main():
    global _to_show_binary, _to_get_pixel, _focus_mode, _map_mode
    global _btn_id_pixel, _btn_id_binary, _btn_id_focus, _gui_ref
    global _btn_id_turn_left, _btn_id_turn_right

    print(app.get_app_config_path())

    cam  = camera.Camera(_image_width, _image_height)
    gui  = GUI()
    _gui_ref = gui

    # 按钮布局
    _btn_id_pixel  = gui.createButton(0, _image_height - _btn_height, _btn_width, _btn_height)
    gui.setItemLabel(_btn_id_pixel, '取阈值')
    gui.setItemCallback(_btn_id_pixel, btn_pressed)

    _btn_id_binary = gui.createButton(_image_width - _btn_width, _image_height - _btn_height, _btn_width, _btn_height)
    gui.setItemLabel(_btn_id_binary, '二值化')
    gui.setItemCallback(_btn_id_binary, btn_pressed)

    focus_x = (_image_width - _btn_width) // 2
    _btn_id_focus = gui.createButton(focus_x, 0, _btn_width, _btn_height)
    gui.setItemLabel(_btn_id_focus, '[ + ]')
    gui.setItemCallback(_btn_id_focus, btn_pressed)

    # 地图模式专用按钮（正常模式下标签为空，不显示）
    turn_btn_w = _btn_width
    _btn_id_turn_left = gui.createButton(_image_width // 3 - turn_btn_w // 2,
                                         _image_height - _btn_height,
                                         turn_btn_w, _btn_height)
    gui.setItemLabel(_btn_id_turn_left, '')
    gui.setItemCallback(_btn_id_turn_left, btn_pressed)
    _btn_id_turn_right = gui.createButton(_image_width * 2 // 3 - turn_btn_w // 2,
                                          _image_height - _btn_height,
                                          turn_btn_w, _btn_height)
    gui.setItemLabel(_btn_id_turn_right, '')
    gui.setItemCallback(_btn_id_turn_right, btn_pressed)

    # 双阈值：生效值 + 临时缓冲区
    active_t1, active_t2 = get_configured_thresholds()
    temp_t1, temp_t2 = list(active_t1), list(active_t2)

    def commit_thresholds():
        nonlocal active_t1, active_t2
        active_t1 = list(temp_t1)
        active_t2 = list(temp_t2)
        set_configured_thresholds(active_t1, active_t2)
        print("阈值已生效: T1", active_t1, "T2", active_t2)

    _last_to_get_pixel = False

    # 持久化双准星坐标
    pick_x1, pick_y1 = -1, -1
    pick_x2, pick_y2 = -1, -1

    # 防重复采样：手指按住不动时不触发取色
    last_tx, last_ty = -1, -1

    # 惯性锁定：记录上一帧左右选定色块的中心坐标
    last_left_cx, last_left_cy = -1, -1
    last_right_cx, last_right_cy = -1, -1

    # ============================================================
    while not app.need_exit():
        # 进出取色模式的瞬间
        if _last_to_get_pixel != _to_get_pixel:
            if not _to_get_pixel:
                commit_thresholds()
                pick_x1, pick_y1 = -1, -1
                pick_x2, pick_y2 = -1, -1
            else:
                pick_x1, pick_y1 = -1, -1
                pick_x2, pick_y2 = -1, -1
        _last_to_get_pixel = _to_get_pixel

        img_raw = cam.read()
        img_dis = img_raw.copy()

        # ---- 地图模式 ----
        if _map_mode:
            draw_map(img_dis)
            gui.run(img_dis)
            continue

        # ---- 二值化预览（仅传入已配置的有效阈值，避免全匹配默认值导致全屏变黑） ----
        if _to_show_binary:
            t1 = temp_t1 if _to_get_pixel else active_t1
            t2 = temp_t2 if _to_get_pixel else active_t2
            valid_ths = []
            if _is_valid_th(t1):
                valid_ths.append(t1)
            if _is_valid_th(t2):
                valid_ths.append(t2)
            if valid_ths:
                img_dis.binary(valid_ths, True)
            img_dis.draw_line(_center_x, 0, _center_x, _image_height,
                              image.Color.from_rgb(255,255,0), 2)

        # ---- 取阈值模式 ----
        if _to_get_pixel:
            tx, ty = gui.get_touch()
            if gui._press_status and tx > 0 and ty > 0:
                if tx != last_tx or ty != last_ty:
                    last_tx, last_ty = tx, ty
                    safe_zone = ty < _image_height - _btn_height - 5
                    if 0 <= tx < _image_width and 0 <= ty < _image_height and safe_zone:
                        rgb = img_raw.get_pixel(tx, ty, True)
                        lab = rgb_to_lab(rgb)
                        if tx < _center_x:
                            pick_x1, pick_y1 = tx, ty
                            temp_t1 = mk_threshold(lab)
                            print("颜色1(左) LAB:", lab, "-> Th:", temp_t1)
                        else:
                            pick_x2, pick_y2 = tx, ty
                            temp_t2 = mk_threshold(lab)
                            print("颜色2(右) LAB:", lab, "-> Th:", temp_t2)
            # 屏幕中轴线
            img_dis.draw_line(_center_x, 0, _center_x, _image_height,
                              image.Color.from_rgb(128,128,128), 1)

            # 双准星持久绘制（红/绿，二值化下也清晰可见）
            if pick_x1 >= 0:
                img_dis.draw_cross(pick_x1, pick_y1, image.COLOR_RED, 12, 2)
                y1 = pick_y1 - 12 if pick_y1 > 12 else pick_y1 + 12
                img_dis.draw_string(pick_x1 + 8, y1, "C1", image.COLOR_RED)
            if pick_x2 >= 0:
                img_dis.draw_cross(pick_x2, pick_y2, image.COLOR_GREEN, 12, 2)
                y2 = pick_y2 - 12 if pick_y2 > 12 else pick_y2 + 12
                img_dis.draw_string(pick_x2 + 8, y2, "C2", image.COLOR_GREEN)

        # ---- 正常检测模式（T1/T2 分开查找，杜绝异色粘连） ----
        else:
            img_dis.draw_line(_center_x, 0, _center_x, _image_height,
                              image.Color.from_rgb(128,128,128), 1)

            left_roi  = (0, 0, _center_x, _image_height)
            right_roi = (_center_x, 0, _image_width - _center_x, _image_height)

            # T1/T2 独立查找，跳过未配置的阈值以节省算力
            left_blobs_raw = []
            if _is_valid_th(active_t1):
                blobs_l_t1 = img_raw.find_blobs([active_t1], roi=left_roi,
                                                area_threshold=60, pixels_threshold=40, merge=True)
                if blobs_l_t1:
                    for b in blobs_l_t1:
                        left_blobs_raw.append((b, True))
            if _is_valid_th(active_t2):
                blobs_l_t2 = img_raw.find_blobs([active_t2], roi=left_roi,
                                                area_threshold=60, pixels_threshold=40, merge=True)
                if blobs_l_t2:
                    for b in blobs_l_t2:
                        left_blobs_raw.append((b, False))

            right_blobs_raw = []
            if _is_valid_th(active_t1):
                blobs_r_t1 = img_raw.find_blobs([active_t1], roi=right_roi,
                                                area_threshold=60, pixels_threshold=40, merge=True)
                if blobs_r_t1:
                    for b in blobs_r_t1:
                        right_blobs_raw.append((b, True))
            if _is_valid_th(active_t2):
                blobs_r_t2 = img_raw.find_blobs([active_t2], roi=right_roi,
                                                area_threshold=60, pixels_threshold=40, merge=True)
                if blobs_r_t2:
                    for b in blobs_r_t2:
                        right_blobs_raw.append((b, False))

            # ---- 处理左半区 ----
            if left_blobs_raw:
                left_pillars = [item for item in left_blobs_raw if is_pillar(item[0])]
                if left_pillars:
                    def left_weight(item):
                        b = item[0]
                        w = b.pixels()
                        if b.w() > b.h():
                            w *= 0.001
                        if last_left_cx >= 0:
                            if abs(b.cx() - last_left_cx) < 40 and abs(b.cy() - last_left_cy) < 40:
                                w *= 1.2
                        return w
                    best_item = max(left_pillars, key=left_weight)
                    best_left, is_t1 = best_item[0], best_item[1]
                    last_left_cx, last_left_cy = best_left.cx(), best_left.cy()
                    auto_correct_map(True, is_t1)
                    label = 'L_T1' if is_t1 else 'L_T2'
                    line_color = image.Color.from_rgb(0,0,0) if is_t1 else image.Color.from_rgb(255,255,255)
                    use_thresh = [active_t1] if is_t1 else [active_t2]
                    roi_l = best_left.rect()
                    img_dis.draw_rect(roi_l[0], roi_l[1], roi_l[2], roi_l[3], image.COLOR_RED, 2)
                    lines_l = img_raw.get_regression(use_thresh, roi=roi_l,
                                                     area_threshold=20, pixels_threshold=20)
                    if lines_l:
                        l = lines_l[0]
                        img_dis.draw_line(l.x1(), l.y1(), l.x2(), l.y2(), line_color, 3)
                        img_dis.draw_string(2, 2,
                            '{} ang:{} mag:{}'.format(label, l.theta(), l.magnitude()), line_color)
                else:
                    last_left_cx, last_left_cy = -1, -1
                    auto_correct_map(True)  # 无柱子 → 白色
            else:
                last_left_cx, last_left_cy = -1, -1
                auto_correct_map(True)  # 无柱子 → 白色

            # ---- 处理右半区 ----
            if right_blobs_raw:
                right_pillars = [item for item in right_blobs_raw if is_pillar(item[0])]
                if right_pillars:
                    def right_weight(item):
                        b = item[0]
                        w = b.pixels()
                        if b.w() > b.h():
                            w *= 0.001
                        if last_right_cx >= 0:
                            if abs(b.cx() - last_right_cx) < 40 and abs(b.cy() - last_right_cy) < 40:
                                w *= 1.2
                        return w
                    best_item = max(right_pillars, key=right_weight)
                    best_right, is_t1 = best_item[0], best_item[1]
                    last_right_cx, last_right_cy = best_right.cx(), best_right.cy()
                    auto_correct_map(False, is_t1)
                    label = 'R_T1' if is_t1 else 'R_T2'
                    line_color = image.Color.from_rgb(0,0,0) if is_t1 else image.Color.from_rgb(255,255,255)
                    use_thresh = [active_t1] if is_t1 else [active_t2]
                    roi_r = best_right.rect()
                    img_dis.draw_rect(roi_r[0], roi_r[1], roi_r[2], roi_r[3], image.COLOR_RED, 2)
                    lines_r = img_raw.get_regression(use_thresh, roi=roi_r,
                                                     area_threshold=20, pixels_threshold=20)
                    if lines_r:
                        l = lines_r[0]
                        img_dis.draw_line(l.x1(), l.y1(), l.x2(), l.y2(), line_color, 3)
                        img_dis.draw_string(best_right.x(), max(2, best_right.y() - 15),
                            '{} ang:{} mag:{}'.format(label, l.theta(), l.magnitude()), line_color)
                else:
                    last_right_cx, last_right_cy = -1, -1
                    auto_correct_map(False)  # 无柱子 → 白色
            else:
                last_right_cx, last_right_cy = -1, -1
                auto_correct_map(False)  # 无柱子 → 白色

        # ---- 双准星持久绘制（取色模式下都画） ----
        gui.run(img_dis)

if __name__ == '__main__':
    main()
