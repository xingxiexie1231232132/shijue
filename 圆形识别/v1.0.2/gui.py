from maix import touchscreen, camera, display, image, time
from maix.image import Image
import math

# ==============================
# 1. GUI 类：按钮和触摸坐标封装
# ==============================

class GUI:
    def __init__(self) -> None:
        """
        初始化 GUI 管理器。

        这个类主要负责三件事：
            1. 读取触摸屏坐标
            2. 判断触摸点是否落在按钮区域内
            3. 在图像上绘制按钮，并把最终图像显示到屏幕
        """

        # background 保存当前要显示的背景图像，也就是主程序传进来的摄像头画面。
        self.background = None

        # items 保存所有按钮的位置和大小。
        # 每个按钮格式为：[x, y, width, height]
        self.items = list()

        # callbacks 保存每个按钮对应的回调函数。
        # 当按钮状态变化时，会自动调用对应回调。
        self.callbacks = list()

        # labels 保存每个按钮上显示的文字。
        self.labels = list()

        # is_active 保存每个按钮的激活状态，控制按钮底色
        self.is_active = list()

        # 最近一次触摸屏读取到的坐标，坐标基于实际显示屏尺寸。
        self.touch_x = 0
        self.touch_y = 0

        # 加载中文字体，避免按钮文字或调试文字显示成方框。
        image.load_font("sourcehansans", "/maixapp/share/font/SourceHanSansCN-Regular.otf")
        # print("fonts:", image.fonts())
        image.set_default_font("sourcehansans")

        # 创建触摸屏和显示屏对象。
        self._ts = touchscreen.TouchScreen()
        self._disp = display.Display()

        # 记录上一帧触摸状态，用来判断“按下/松开”是否发生变化。
        self._last_pressed = 0

    # ==============================
    # 2. 内部函数：判断触摸点是否在按钮内
    # ==============================

    def _is_in_item(self, item_id: int, x: int, y: int) -> bool:
        """
        判断触摸点是否落在指定按钮区域内。

        参数：
            item_id：按钮编号
            x, y：触摸屏坐标，基于显示屏尺寸

        返回：
            True：触摸点在按钮区域内
            False：触摸点不在按钮区域内
        """

        if item_id >= len(self.items) or self.background == None:
            return False

        item_pos = self.items[item_id]

        # 主程序中的按钮坐标是基于 background 图像尺寸的。
        # 触摸屏读取到的 x、y 是基于实际显示屏尺寸的。
        # 如果图像显示时做了缩放或留边，就必须先把按钮区域映射到显示屏坐标。
        item_disp_pos = image.resize_map_pos(
            self.background.width(),
            self.background.height(),
            self._disp.width(),
            self._disp.height(),
            image.Fit.FIT_CONTAIN,
            item_pos[0],
            item_pos[1],
            item_pos[2],
            item_pos[3]
        )

        if x > item_disp_pos[0] and x < (item_disp_pos[0] + item_disp_pos[2]) and y > item_disp_pos[1] and y < (item_disp_pos[1] + item_disp_pos[3]):
            return True
        else:
            return False

    # ==============================
    # 3. 创建和配置按钮
    # ==============================

    def createButton(self, x: int, y: int, width: int, height: int) -> int:
        '''
        创建一个按钮组件。

        参数：
            x, y：按钮左上角坐标，基于摄像头图像坐标
            width, height：按钮宽度和高度，单位是像素

        返回：
            item_id：按钮编号，后续设置文字或回调时会用到
        '''

        item_id = len(self.items)
        self.items.append([x, y, width, height])
        self.callbacks.append(None)
        self.labels.append(None)
        self.is_active.append(False)
        return item_id

    def setItemCallback(self, item_id: int, cb) -> None:
        '''
        设置按钮回调函数。

        回调函数会在按钮触摸状态变化时被调用，格式为：
            callback(item_id, state)

        其中：
            item_id：按钮编号
            state：触摸状态，通常 1 表示按下，0 表示松开
        '''

        if item_id >= len(self.items):
            return
        self.callbacks[item_id] = cb

    def setItemLabel(self, item_id: int, label: str) -> None:
        if item_id >= len(self.items):
            return
        self.labels[item_id] = label

    def setItemActive(self, item_id: int, active: bool) -> None:
        """设置按钮激活状态，控制底色变化"""
        if item_id < len(self.items):
            self.is_active[item_id] = active

    # ==============================
    # 4. 获取触摸点在图像中的坐标
    # ==============================

    def get_touch(self) -> tuple:
        '''
        返回最近一次触摸点在摄像头图像坐标系中的位置。

        说明：
            self.touch_x、self.touch_y 是显示屏坐标；
            主程序处理图像时需要的是图像坐标，所以这里要做反向映射。
        '''

        if self.background == None:
            return (0, 0)

        x, y = image.resize_map_pos_reverse(
            self.background.width(),
            self.background.height(),
            self._disp.width(),
            self._disp.height(),
            image.Fit.FIT_CONTAIN,
            self.touch_x,
            self.touch_y
        )

        # 防止触摸点落在图像显示区域外时出现负数坐标。
        x = x if x >= 0 else 0
        y = y if y >= 0 else 0
        return (x, y)

    # ==============================
    # 5. 刷新 GUI 和显示图像
    # ==============================

    def run(self, background: Image) -> None:
        """
        刷新 GUI。

        参数：
            background：当前要显示的图像，通常是摄像头读取到的一帧画面。
        """

        self.background = background
        self.touch_x, self.touch_y, pressed = self._ts.read()

        # ------------------------------
        # 5.1 处理触摸状态变化
        # ------------------------------

        # 只有触摸状态发生变化时才触发按钮回调。
        # 这样可以避免手指一直按住时，回调函数在每一帧都被重复调用。
        if self._last_pressed != pressed:
            self._last_pressed = pressed

            for item_id in range(len(self.items)):
                if self._is_in_item(item_id, self.touch_x, self.touch_y):
                    if self.callbacks[item_id] != None:
                        self.callbacks[item_id](item_id, pressed)
                    break

        # ------------------------------
        # 5.2 绘制按钮和按钮文字
        # ------------------------------

        for item_id in range(len(self.items)):
            if self.labels[item_id] != None:
                label_size = image.string_size(self.labels[item_id])
            else:
                label_size = image.string_size("")

            # 让文字尽量显示在按钮中心。
            label_x = (self.items[item_id][0] + (self.items[item_id][2] - label_size.width()) // 2) if self.items[item_id][2] > label_size.width() else self.items[item_id][0]
            label_y = (self.items[item_id][1] + (self.items[item_id][3] - label_size.height()) // 2) if self.items[item_id][3] > label_size.height() else self.items[item_id][1]

            # 取阈值/二值化按钮：无边框仅文字；专注按钮激活时灰底
            if self.is_active[item_id]:
                self.background.draw_rect(
                    self.items[item_id][0], self.items[item_id][1],
                    self.items[item_id][2], self.items[item_id][3],
                    image.COLOR_GRAY, -1)

            if self.labels[item_id] != None:
                self.background.draw_string(label_x, label_y, self.labels[item_id], image.COLOR_BLACK)

        # ------------------------------
        # 5.3 显示最终图像
        # ------------------------------
        self._disp.show(self.background)


# ==============================
# 6. 独立运行时的 GUI 测试程序
# ==============================

if __name__ == '__main__':

    def btn_pressed(btn_id, state):
        print('item {} state: {}'.format(btn_id, state))

    disp_width = 320
    disp_height = 240

    # 手动设置较小分辨率，避免默认分辨率过大导致显示和计算负担增加。
    cam = camera.Camera(disp_width, disp_height)

    gui = GUI()

    btn_id1 = gui.createButton(0, 0, 60, 40)
    gui.setItemLabel(btn_id1, 'AA')
    gui.setItemCallback(btn_id1, btn_pressed)

    btn_id2 = gui.createButton(0, disp_height - 40, 60, 40)
    gui.setItemLabel(btn_id2, 'BB')
    gui.setItemCallback(btn_id2, btn_pressed)

    btn_id3 = gui.createButton(disp_width - 60, disp_height - 40, 60, 40)
    gui.setItemLabel(btn_id3, 'CC')
    gui.setItemCallback(btn_id3, btn_pressed)

    btn_id4 = gui.createButton(disp_width - 60, 0, 60, 40)
    gui.setItemLabel(btn_id4, 'DD')
    gui.setItemCallback(btn_id4, btn_pressed)

    while True:
        img = cam.read()
        gui.run(img)
