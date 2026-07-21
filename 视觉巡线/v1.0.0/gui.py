# 从 maix 库中导入需要用到的模块
# touchscreen：触摸屏模块，用来读取手指点击位置
# camera：摄像头模块，用来读取摄像头画面
# display：显示屏模块，用来把图像显示到屏幕上
# image：图像模块，用来画矩形、写文字、加载字体等
# time：时间模块，本程序中暂时没有用到
from maix import touchscreen, camera, display, image, time

# 导入 Image 类型，主要用于函数参数的类型标注
from maix.image import Image

# 导入数学库，本程序中暂时没有用到
import math

# 定义一个类，类名叫 GUI
    # 在 Python 里，类名一般推荐使用大驼峰命名法，也就是每个单词首字母大写
class GUI:

    # 定义一个叫 GUI 的类，并规定：当创建 GUI 对象时，自动执行 __init__() 里面的代码
    # self 表示：当前这个对象自己(约定俗称)。例如：
        # self.name = "我的GUI" <=> gui.name = "我的GUI"
        # "."的后面叫做 属性名，也可以理解为这个对象里面的一个“变量”
    # 原因：Python 调用类方法时，会自动把调用这个方法的对象作为第一个参数传进去
    def __init__(self) -> None:

        self.background = None   # 背景图像，初始为空
        self.items = list()      # 保存按钮的位置和大小，list为列表
        self.callbacks = list()  # 保存按钮点击后的执行函数
        self.labels = list()     # 保存按钮显示的文字

        self.touch_x = 0         # 当前触摸点的 x 坐标
        self.touch_y = 0         # 当前触摸点的 y 坐标

        # image.load_font() 这是 MaixPy 里面 image 模块提供的函数。它的作用是：加载一个字体文件
        # 因为默认字体可能不支持中文，所以如果你想在屏幕上显示中文，比如：img.draw_string(0, 0, "开始识别")，就需要先加载一个支持中文的字体
        image.load_font(
            "sourcehansans", # 这是你给这个字体起的名字
            "/maixapp/share/font/SourceHanSansCN-Regular.otf" # 这是字体文件在 MaixCAM 系统里的位置
        )

        # 把 "sourcehansans" 这个字体设置成默认字体。
        image.set_default_font("sourcehansans")

        # self. 里面的这个"."的意思是：访问某个对象里面的东西
        # _ts：给当前 GUI 对象创建一个 _ts 属性，用来保存触摸屏对象。
            # 前面的 _ 是一种 Python 命名习惯。通常表示：这个变量主要给类内部自己使用，外部不要随便访问或修改
        self._ts = touchscreen.TouchScreen()

        # 创建显示屏对象。后面可以通过 self._disp.show(img) 显示图像
        self._disp = display.Display()

        # 保存上一次触摸屏是否按下的状态。作用：判断触摸状态有没有发生变化
        self._last_pressed = 0

    # _is_in_item 这是函数名，也就是方法名。这个函数主要给 GUI 类内部自己用，不建议外部直接调用。
        # self  表示当前这个 GUI 对象
        # item_id: int 表示参数 item_id，建议它是 int 类型
        # x 是横坐标，y 是纵坐标
    # 如果要调用这个函数，写法可以参照：self._is_in_item(item_id, x, y)。例如：
        # self._is_in_item(item_id, x, y)，意思是：判断第 0 个按钮，是否包含坐标点 (120, 80)
        # 如果点在按钮里面：result = True；否则 result = False

    def _is_in_item(self, item_id: int, x: int, y: int) -> bool:
        '''
        判断当前触摸点是否落在某一个按钮区域内。

        参数说明：
        self:
            当前 GUI 对象本身。

        item_id:
            按钮编号。
            因为 self.items 里面可能保存了很多按钮，
            所以 item_id 用来指定要判断的是第几个按钮。

            例如：
            self.items = [
                [20, 30, 100, 40],     # 第 0 个按钮
                [150, 30, 100, 40],    # 第 1 个按钮
                [20, 100, 230, 50]     # 第 2 个按钮
            ]

            item_id = 0 表示判断第 0 个按钮
            item_id = 1 表示判断第 1 个按钮
            item_id = 2 表示判断第 2 个按钮

        x:
            当前触摸点在屏幕上的 x 坐标。

        y:
            当前触摸点在屏幕上的 y 坐标。

        返回值：
            True:
                表示触摸点在按钮里面，也就是点中了按钮。

            False:
                表示触摸点不在按钮里面，也就是没有点中按钮。

        -> bool:
            这是类型提示，表示这个函数最终会返回布尔值：
            True 或 False。
        '''

        # ------------------------------------------------------------
        # 第一步：如果按钮编号超出范围，或者背景图像还没有设置，就直接返回 False
        # ------------------------------------------------------------
        if item_id >= len(self.items) or self.background == None:
            return False

        # ------------------------------------------------------------
        # 第二步：从 items 列表中取出指定按钮的位置和大小
        # ------------------------------------------------------------

        # 从列表中获取对应的按钮数据
        item_pos = self.items[item_id]
        
        # ------------------------------------------------------------
        # 第三步：把按钮坐标从“原图坐标”换算成“屏幕坐标”
        # ------------------------------------------------------------
        # 你的按钮坐标 item_pos 是基于背景图像 background 的。但是用户用手触摸屏幕时，触摸屏返回的是“屏幕坐标”。
        #
        # 问题是：
            # 背景图像的尺寸 和 屏幕的尺寸 不一定一样。
        #
        # 例如：
            # 摄像头图像大小可能是：320 × 240。屏幕大小可能是：552 × 368
        #
        # 如果直接拿原图坐标去和屏幕触摸坐标比较，就可能判断错误。所以这里要使用 image.resize_map_pos()，把按钮在原图中的位置，换算成按钮在屏幕上的实际显示位置

        item_disp_pos = image.resize_map_pos(
            self.background.width(),      # 原始背景图像的宽度。例如 background 是摄像头读取到的画面，它的宽度可能是 320。
            self.background.height(),     # 原始背景图像的高度。例如 background 的高度可能是 240。
            self._disp.width(),           # 屏幕的宽度。self._disp 是显示屏对象。self._disp.width() 用来获取屏幕宽度。
            self._disp.height(),          # 屏幕的高度。self._disp.height() 用来获取屏幕高度。
            image.Fit.FIT_CONTAIN,        # 图像完整显示，不裁剪
            item_pos[0],                  # 按钮左上角 x。item_pos[0] 对应 [x, y, width, height] 里的 x
            item_pos[1],                  # 按钮左上角 y
            item_pos[2],                  # 按钮宽度
            item_pos[3]                   # 按钮高度
        )

        # image.resize_map_pos() 执行之后，item_disp_pos 里面保存的是按钮在屏幕上的实际位置和大小。它的格式仍然是：[x, y, width, height]
        # 但是注意：
            #
            # item_pos 是原图坐标中的按钮位置。
            # item_disp_pos 是屏幕坐标中的按钮位置。
        # 例如：
            #
            # 原图中的按钮位置可能是：
            # item_pos = [20, 30, 100, 40]
            #
            # 显示到屏幕后，按钮可能被缩放，于是变成：
            # item_disp_pos = [35, 46, 172, 64]
        # 后面判断触摸点时，必须使用 item_disp_pos，因为触摸点 x、y 是屏幕坐标。

        # ------------------------------------------------------------
        # 第四步：判断触摸点是否在按钮矩形区域内
        # ------------------------------------------------------------
        # 假设按钮在屏幕上的位置是：item_disp_pos = [20, 30, 100, 40]
        #
        # 那么：
            # item_disp_pos[0] = 20   按钮左上角 x 坐标
            # item_disp_pos[1] = 30   按钮左上角 y 坐标
            # item_disp_pos[2] = 100  按钮宽度
            # item_disp_pos[3] = 40   按钮高度
        #
        # 所以按钮的四条边界是：
            # 左边界：20
            # 右边界：20 + 100 = 120
            # 上边界：30
            # 下边界：30 + 40 = 70
        #
        # 一个点要在按钮里面，必须同时满足：
            # 触摸点 x > 左边界
            # 触摸点 x < 右边界
            # 触摸点 y > 上边界
            # 触摸点 y < 下边界

        if (
            # item_disp_pos[0] 是按钮左上角 x 坐标，item_disp_pos[2] 是按钮宽度
            x > item_disp_pos[0]                              
            and x < (item_disp_pos[0] + item_disp_pos[2])     

            # item_disp_pos[1] 是按钮左上角 y 坐标，item_disp_pos[3] 是按钮高度
            and y > item_disp_pos[1]                          
            and y < (item_disp_pos[1] + item_disp_pos[3])     
        ):
            return True     # 点在按钮里面
        else:
            return False    # 点不在按钮里面


    def createButton(self, x: int, y: int, width: int, height: int) -> int:
        '''
        创建一个按钮，并把按钮的信息保存到 GUI 对象中。

        参数说明：
            x:
                按钮左上角的 x 坐标。

            y:
                按钮左上角的 y 坐标。

            width:
                按钮的宽度。

            height:
                按钮的高度。

        返回值：
            item_id:
                当前创建的按钮编号。
                后面可以通过这个编号来设置按钮文字、绑定点击函数等。
        '''

        # ------------------------------------------------------------
        # 1. 生成当前按钮的编号
        # ------------------------------------------------------------

        # self.items 是一个列表，用来保存所有按钮的位置和大小。len(self.items) 表示当前已经有多少个按钮
        item_id = len(self.items)

        # ------------------------------------------------------------
        # 2. 保存按钮的位置和大小
        # ------------------------------------------------------------

        # 把按钮的位置和大小保存到 self.items 列表中
            # [x, y, width, height] 是一个列表，用来表示一个按钮的矩形区域
        self.items.append([x, y, width, height])

        # ------------------------------------------------------------
        # 3. 给这个按钮预留一个回调函数位置
        # ------------------------------------------------------------

        # self.callbacks 是一个列表，用来保存每个按钮被点击后要执行的函数
        # callback 的意思是“回调函数”。简单理解：
            # 按钮被点击后，要自动执行哪个函数，就把那个函数保存到 self.callbacks 里面
        # 这里先 append(None)，表示：
            # 当前这个按钮暂时还没有绑定点击函数
        # 为什么要先放一个 None？
            # 因为 self.items、self.callbacks、self.labels
            # 这三个列表是按照相同编号一一对应的。比如：
                # self.items[0]      表示第 0 个按钮的位置和大小
                # self.callbacks[0]  表示第 0 个按钮点击后执行的函数
                # self.labels[0]     表示第 0 个按钮显示的文字
            # 所以创建按钮时，即使暂时没有回调函数，也要先放一个 None 占位
        self.callbacks.append(None)

        # ------------------------------------------------------------
        # 4. 给这个按钮预留一个文字标签位置
        # ------------------------------------------------------------

        # self.labels 是一个列表，用来保存每个按钮上显示的文字
        self.labels.append(None)

        return item_id


    def setItemCallback(self, item_id: int, cb) -> None:
        '''
        给指定按钮设置回调函数。

            参数说明：
                item_id:
                    按钮编号。

                    每创建一个按钮，都会得到一个 item_id。
                    这个 item_id 用来找到对应的按钮。

                cb:
                    callback 的缩写，表示“回调函数”。

                    简单理解：
                    当按钮被点击时，要执行的函数。

            返回值：
                None:
                    表示这个函数不返回结果。
                    它只是完成“设置回调函数”这个操作。
        '''

        # ------------------------------------------------------------
        # 1. 判断按钮编号是否合法
        # ------------------------------------------------------------
        if item_id >= len(self.items):
            return # 结束函数，不继续执行

        # ------------------------------------------------------------
        # 2. 给指定按钮绑定回调函数
        # ------------------------------------------------------------

        # 把 cb 这个函数，绑定到编号为 item_id 的按钮上。
        self.callbacks[item_id] = cb

    def setItemLabel(self, item_id: int, label: str) -> None:
        '''
        设置指定按钮上显示的文字。

        参数说明：
            item_id:
                按钮编号。
                用来指定要修改哪一个按钮的文字。

            label:
                要显示在按钮上的文字内容。
                例如："开始"、"停止"、"拍照" 等。

        返回值：
            None:
                表示这个函数不返回结果，只负责设置按钮文字。
        '''

        # 如果按钮编号不存在，直接退出
        if item_id >= len(self.items):
            return

        # 保存按钮文字
        self.labels[item_id] = label

    # 元组 tuple 一旦创建，里面的数据不能改；数组/列表一般可以改。
    def get_touch(self) -> tuple:
        '''
        获取最近一次触摸点的位置。

        注意：
            触摸屏返回的坐标通常是“屏幕坐标”。
            但是 GUI 按钮、背景图等很多内容可能是按“原图坐标”设计的。

            所以这里会把触摸点坐标从“屏幕坐标”
            反向换算回“原图坐标”。

        返回值：
            返回一个元组 (x, y)

            x:触摸点在原图中的 x 坐标
            y:触摸点在原图中的 y 坐标
        '''

        # ------------------------------------------------------------
        # 1. 如果当前还没有背景图像，就无法进行坐标换算
        # ------------------------------------------------------------

        # 如果 self.background == None，说明当前还没有图像
            # 因为后面要用背景图像的宽度和高度来换算坐标，所以如果背景图不存在，就直接返回默认坐标 (0, 0)
        if self.background == None:
            return (0, 0)

        # ------------------------------------------------------------
        # 2. 把触摸屏坐标反向映射回原图坐标
        # ------------------------------------------------------------
        
        # 它们表示的是：手指点在“屏幕”上的位置。但是背景图像显示到屏幕上时，可能被缩放过。
            # 例如：
            # 原图大小是 320 × 240
            # 屏幕大小是 552 × 368
            # 显示时图像可能会被放大、缩小，甚至左右或上下有黑边。所以不能直接把 self.touch_x、self.touch_y 当成原图坐标使用
        x, y = image.resize_map_pos_reverse(
            # 调用 background 对象的 width() 方法，得到图像宽度
                # 方法的后面有()，本质上就是属于对象的函数
            self.background.width(),      # 原图宽度
            self.background.height(),     # 原图高度
            self._disp.width(),           # 屏幕宽度
            self._disp.height(),          # 屏幕高度
            image.Fit.FIT_CONTAIN,        # 图像完整显示，不裁剪
            self.touch_x,                 # 当前触摸点 x
            self.touch_y                  # 当前触摸点 y
        )

        # ------------------------------------------------------------
        # 3. 将映射结果夹紧到原图的合法坐标范围
        # ------------------------------------------------------------

        # FIT_CONTAIN 模式可能在屏幕边缘留下黑边；点击黑边或图像边界时，
        # 反向映射结果可能小于 0，也可能刚好等于图像宽度/高度。
        # 图像坐标从 0 开始，因此 320 x 240 图像的有效范围实际是：
        #   x = 0~319，y = 0~239，而不是 x = 0~320、y = 0~240。
        # 同时限制上下界，可避免 img.get_pixel() 因坐标越界而读取失败。
        max_x = self.background.width() - 1
        max_y = self.background.height() - 1
        x = max(0, min(x, max_x))
        y = max(0, min(y, max_y))

        return (x, y)

    # : Image 是类型提示。意思是告诉你：这个参数最好是 Image 类型
    def run(self, background: Image) -> None:
        '''
        运行一次 GUI 更新流程。

        参数：
            background:
                当前要显示的背景图像。
                通常是摄像头读取到的一帧画面。

        功能：
            1. 保存当前背景图像
            2. 读取触摸屏状态
            3. 判断是否点击了某个按钮
            4. 如果点击了按钮，就执行对应回调函数
            5. 把所有按钮和文字画到背景图像上
            6. 把最终图像显示到屏幕上
        '''
        # ------------------------------------------------------------
        # 1. 保存当前背景图像
        # ------------------------------------------------------------

        # 把传进来的 background 保存到 self.background。
            # 这样类里面其他函数也可以使用这张图像，
            # 比如 _is_in_item() 里面需要用它的宽度和高度来做坐标换算。
        self.background = background

        # ------------------------------------------------------------
        # 2. 读取触摸屏状态
        # ------------------------------------------------------------

        # self._ts 是触摸屏对象。
        #
        # self._ts.read() 会读取当前触摸屏状态，
        # 一般返回三个值：
            # touch_x：当前触摸点的 x 坐标
            # touch_y：当前触摸点的 y 坐标
            # pressed：当前是否按下
        #
        # pressed 通常可以理解为：
            # 0：没有按下
            # 1：正在按下
        self.touch_x, self.touch_y, pressed = self._ts.read() # self._ts 是触摸屏对象 .read 再调用 _ts 中的 read()方法 → 这是面向对象语言的写法

        # ------------------------------------------------------------
        # 3. 判断触摸状态是否发生变化
        # ------------------------------------------------------------

        # self._last_pressed 保存的是上一次的按下状态。
        # pressed 是当前这一次读取到的按下状态。如果两者不一样，说明触摸状态发生了变化
        #
        # 例如：
            # 上一次 pressed = 0，这一次 pressed = 1。说明手指刚刚按下
            # 或者：上一次 pressed = 1，这一次 pressed = 0。说明手指刚刚松开
        # 只有状态发生变化时才处理按钮事件，可以避免手指一直按住时，按钮被反复触发

        if self._last_pressed != pressed:

            # 更新上一次触摸状态。这样下一次 run() 被调用时， 就可以拿新的 pressed 状态和这次状态进行比较
            self._last_pressed = pressed

            # --------------------------------------------------------
            # 4. 遍历所有按钮，判断当前触摸点是否点中了按钮
            # --------------------------------------------------------
            
            # len(self.items) 表示当前按钮数量。range(len(self.items)) 会生成：0, 1, 2, ... 也就是依次遍历每一个按钮编号
            for id in range(len(self.items)):

                # 判断当前触摸点是否在第 id 个按钮内部
                    # self.touch_x 和 self.touch_y 是屏幕上的触摸坐标
                    # _is_in_item() 会判断这个坐标是否落在按钮区域内
                # 程序的判断逻辑就是：拿同一个手指坐标。依次和每一个按钮区域比较。看看它落在哪个按钮里面

                if self._is_in_item(id, self.touch_x, self.touch_y):

                    # 如果这个按钮设置了回调函数，才执行
                    # self.callbacks[id] 里面保存的是第 id 个按钮：
                        # 被点击后需要执行的函数。
                        # 如果它是 None，说明这个按钮暂时没有绑定函数。
                    if self.callbacks[id] != None:

                        # 执行回调函数：
                            # id：按钮编号
                            # pressed：按下状态
                        # 注意：此时是定义好后续如果要使用这个回调函数要传进去这两个参数 → id、pressed
                        self.callbacks[id](id, pressed)

                    # 找到一个被点击的按钮后，就退出循环
                    # 这样一次触摸只会触发一个按钮
                    break

        # 按照 self.items 这个列表的长度，从 0 开始依次生成编号，然后逐个循环处理。
            # 依次取出第 0 个按钮
            # 依次取出第 1 个按钮
            # 依次取出第 2 个按钮
        for id in range(len(self.items)):

            # ⚠️ 边框由调用方在 gui.run() 之前自行绘制（圆形/矩形底色），
            #     此处不再绘制红色矩形边框，仅保留文字。

            # 如果这个按钮设置了文字，才绘制文字。这样可以避免 label 为 None 时出错
            if self.labels[id] != None:

                label_size = image.string_size(self.labels[id])

                # -------------------------------
                # 计算文字的 x 坐标，使文字水平居中
                # -------------------------------

                # 如果按钮宽度大于文字宽度，说明文字可以放进按钮里，并且可以居中
                    # 文字的 x 坐标 = 按钮左上角 x 坐标（self.items[id][0]） + 左右剩余空白的一半（self.items[id][2] - label_size.width()）
                    # //2 在Python中是除以 2 的意思
                if self.items[id][2] > label_size.width():
                    label_x = self.items[id][0] + (
                        self.items[id][2] - label_size.width()
                    ) // 2

                # 如果文字比按钮还宽，就从按钮左边开始画
                else:
                    label_x = self.items[id][0]

                # -------------------------------
                # 计算文字的 y 坐标，使文字垂直居中
                # -------------------------------

                # self.items[id][3] 是按钮高度。如果按钮高度大于文字高度，说明文字可以垂直居中
                
                if self.items[id][3] > label_size.height():
                    label_y = self.items[id][1] + (
                        self.items[id][3] - label_size.height()
                    ) // 2

                # 如果文字比按钮还高，就从按钮上边开始画
                else:
                    label_y = self.items[id][1]

                # 在背景图像 self.background 上画文字
                self.background.draw_string(
                    label_x,               # 文字 x 坐标
                    label_y,               # 文字 y 坐标
                    self.labels[id],       # 要显示的文字
                    image.COLOR_WHITE      # 文字颜色：白色
                )

        # 把最终图像显示到屏幕上。此时图像中已经包含：摄像头画面 + 按钮边框 + 按钮文字
        self._disp.show(self.background)

# 程序入口。如果当前文件是被直接运行的，就执行下面的代码
if __name__ == '__main__':

    # 这里定义了一个函数，名字叫：btn_pressed。它是按钮被点击时要执行的函数，也就是回调函数
        # 两个参数 btn_id 和 state 和前面定义 self.callbacks[id](id, pressed) 形式上要对应
        # 打印按钮编号和按钮状态。例如，如果点击了第 0 个按钮，状态是 1，可能会输出：item 0 state: 1
    def btn_pressed(btn_id, state):
        print('item {} state: {}'.format(btn_id, state))

    disp_width = 320

    disp_height = 240

    # 创建一个摄像头对象，并保存到变量 cam 里
    cam = camera.Camera(disp_width, disp_height)

    # 根据 GUI 这个类，创建一个对象。然后把创建出来的对象保存到变量 gui 中
        # class GUI 是“设计图纸”
        # gui = GUI()则是“按照图纸造出一个真实对象”
    gui = GUI()

    # 调用 gui 对象的 createButton() 方法，创建第一个按钮
        # createButton 这个函数的返回值为 item_id（按钮序号）
    btn_id1 = gui.createButton(0, 0, 60, 40)

    # 给第一个按钮设置显示文字：AA
    gui.setItemLabel(btn_id1, 'AA')

    # 给第一个按钮绑定回调函数。当第一个按钮被点击时，执行 btn_pressed 这个函数
    gui.setItemCallback(btn_id1, btn_pressed)

    # 调用 gui 对象的 createButton() 方法，创建第二个按钮
    btn_id2 = gui.createButton(0, disp_height - 40, 60, 40)

    # 给第二个按钮设置显示文字：BB
    gui.setItemLabel(btn_id2, 'BB')

    # 给第二个按钮绑定回调函数。当第二个按钮被点击时，执行 btn_pressed 这个函数
    gui.setItemCallback(btn_id2, btn_pressed)

    # 后面同理
    btn_id3 = gui.createButton(disp_width - 60, disp_height - 40, 60, 40)

    gui.setItemLabel(btn_id3, 'CC')

    gui.setItemCallback(btn_id3, btn_pressed)

    btn_id4 = gui.createButton(disp_width - 60, 0, 60, 40)

    gui.setItemLabel(btn_id4, 'DD')

    gui.setItemCallback(btn_id4, btn_pressed)

    # 因为 gui.run(img) 放在 while True 无限循环里面，所以它会一直重复执行
    # 也就是说：self.touch_x, self.touch_y, pressed = self._ts.read() 不是只执行一次，而是每一轮循环都会执行一次
    # 可以理解成：
        # 第 1 次循环：读取一次触摸坐标
        # 第 2 次循环：再读取一次触摸坐标
        # 第 3 次循环：再读取一次触摸坐标
        # 第 4 次循环：再读取一次触摸坐标…
    while True:
        img = cam.read()
        gui.run(img)
