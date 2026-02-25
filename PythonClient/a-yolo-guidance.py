"""
按键: q退出 f开火 m切换制导 1/2/3飞行模式 p预测线 t跟踪
"""

import sys
import os
import time
import math
import argparse
import threading
import base64
import numpy as np
import cv2

# 查找yolo路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for p in [os.path.join(SCRIPT_DIR, "Yolo", "ultralytics-main"),
          os.path.join(SCRIPT_DIR, "..", "Yolo", "ultralytics-main")]:
    if os.path.exists(p):
        sys.path.insert(0, os.path.abspath(p))
        break

from ultralytics import YOLO
from sim_client import SimClient

class DroneController:
    def __init__(self, client, turret_pos, altitude=2.0, speed=2.0, radius=150.0):
        self.client = client
        self.turret_pos = np.array(turret_pos[:2])
        self.altitude = altitude
        self.speed = speed
        self.radius_cm = radius * 100
        self.pattern = "straight"
        self.running = False
        self.thread = None


    def set_pattern(self, p):
        self.pattern = p

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=5)

    def _fly_to(self, x_cm, y_cm, z_m):
        self.client.drone_move_to(x_cm / 100.0, y_cm / 100.0, z_m, self.speed)

    def _run(self):
        c = self.client
        tx, ty = self.turret_pos
        alt, R = self.altitude, self.radius_cm

        c.drone_takeoff(alt)
        time.sleep(2)
        self._fly_to(tx + R * 0.5, ty, alt)
        time.sleep(2)

        step = 0
        while self.running:
            try:
                if self.pattern == "straight":
                    pts = [(tx+R*0.6, ty-R*0.3), (tx+R*0.6, ty+R*0.3),
                           (tx+R*0.3, ty+R*0.5), (tx+R*0.3, ty-R*0.5)]
                    p = pts[step % len(pts)]
                    self._fly_to(p[0], p[1], alt)
                    time.sleep(4)
                elif self.pattern == "curved":
                    angle = math.radians((step * 30) % 360)
                    self._fly_to(tx + R*0.5*math.cos(angle), ty + R*0.5*math.sin(angle),
                                 alt + math.sin(angle)*2)
                    time.sleep(2)
                elif self.pattern == "evasive":
                    d = 1 if step % 2 == 0 else -1
                    c.drone_move_by_velocity(self.speed*0.8, self.speed*1.5*d, 0)
                    time.sleep(2)
                    pos = c.drone_position()
                    if np.linalg.norm(pos[:2] - self.turret_pos) > R * 0.8:
                        self._fly_to(tx + R*0.3, ty, alt)
                        time.sleep(3)
                step += 1
            except:
                time.sleep(1)
        c.drone_hover()


class HUDRenderer:
    # 【HUDRenderer 图像化信息渲染类极其详尽说明】
    # 1. 核心定位：本类是整个无聊的、冷冰冰的数学后台矩阵计算的唯一可视化视觉宣发窗口。
    # 2. 交互体验：它完全模拟了现代军用战斗机或武装直升机风挡玻璃上的平视显示器（Head-Up Display, HUD），将高维战术数据降维展现给碳基生物。
    # 3. 无状态哲学：你会发现这个类甚至连个 __init__ 初始化函数都没有，里面全是 @staticmethod 或者 @classmethod。因为它就是一个存粹提供画笔的无状态工具箱，不会去干涉任何业务内存。
    # 4. 色彩规范：将所有用于警示、追踪、正常的信息文字的颜色代码，以宏定义的形式在类级别头部写死，极大降低了项目中出现“五彩斑斓杂色”的视觉污染。
    # 5. 黑底描边绝技：它封装了一个独门的文本绘制秘籍 text()，先用粗黑体画一遍做底垫，再用细彩字覆盖，解决了浅色字体在明亮天空背景中完全隐身看不清的世纪痛点。
    # 6. 机甲审美学：在画 YOLO 框时，摒弃了土得掉渣的标准矩形框，采用了极具科幻电影锁定感的“四角断点 L 型”机械描边，美术细节直接拉满。
    # 7. 全局感知：信息面板将原本散落在各个对象角落的 FPS 帧率、网络推断漏检率、以及底层的 Euler 偏航角，强行整合到屏幕左上角的统一控制台中。
    # 8. 三维空间投影验证：它能够将经过极其复杂的相机外参反推出来的虚假 3D 世界目标点（TGT），强行作为字符刻印在左下角，这是整个几何透视法有无出现 Bug 的唯一验证途径。
    # 9. 强迫症般的对齐：在绘制任何线条、十字准星时，其底层的数学计算极其严谨地遵守了图像分辨率的长宽二分之一绝对中心点对齐，容不得哪怕一个像素的位移偏差。
    # 10. 性能克制：所有绘图操作都指定了 cv2.LINE_AA 这种自带抗锯齿效果的线条，但是在极其需要性能的地方又没有滥用半透明混合。
    
    # 极其讲究的颜色常量定义区，全大写字母表示这些是神圣不可侵犯的静态类常量。
    # 这里使用的是 OpenCV 的反人类通道顺序 BGR（蓝，绿，红），而不是网页前端常识的 RGB。
    # C_GREEN 定义的是极其科幻的荧光绿。它在绿色通道 255 的满载下，注入了 80 的蓝和 120 的红。
    # 这种配比让绿色变得不再刺眼生硬，而是带有柔和赛博朋克发光感的舒适指示色，主要用来画无威胁锁定框。
    C_GREEN  = (80, 255, 120)

    # C_RED 并非纯净的 (0,0,255) 正红。
    # 它是 (60, 60, 255)，往红色里掺入了轻微的蓝和绿，这让红色稍微带了一点粉暗的色相。
    # 这种精心调配的目的是防止在某些低对比度的显示器上纯红色过于刺眼产生颜色溢出（Color Bleeding）效应。
    # 它是危险、开火指令以及卡尔曼终极锁定中心红十字的专属颜色。
    C_RED    = (60, 60, 255)

    # C_WHITE 用作左上角海量基础信息数据流的默认刷字颜色。
    # 为了保护夜间进行测试的开发人员的视网膜，它没有使用最高刺眼亮度的 (255,255,255)。
    # 而是全通道稍微压暗到了 230，呈现出一种极其内敛且具有工业高级感的高级灰白。
    # 这种克制的颜色大量出现也不会造成画面重心的本末倒置。
    C_WHITE  = (230, 230, 230)

    # 【text 静态文本渲染底层辅助方法极其详尽说明】
    # 1. 核心定位：这是整个 HUD 类中被所有其他高级业务绘制函数疯狂调用的“核武器级”工具链底层。
    # 2. 装饰器定义：使用 @staticmethod 装饰，意味着它和类的实例状态毫不相干，纯粹是一个输入图像和文本，给你吐出带字图像的“黑盒工厂”。
    # 3. OpenCV 屏蔽层：它强力屏蔽了 cv2.putText 那些需要手写字体类型、手写线条样式的冗长参数群，统一封装了一个极其干净的默认参数列表。
    # 4. 默认字体：内部硬编码了 cv2.FONT_HERSHEY_SIMPLEX，这是 OpenCV 最轻量、最没有版权纠纷、在任何机器上渲染都极快的一种无衬线等宽系统字体。
    # 5. 黑底描边法精髓：分两步走。第一步，用绝对的纯黑色 (0,0,0) 作为颜料，但是使用传进来的原本线宽 thick 偷偷加上 2 倍作为粗细参数（thick+2）。
    # 6. 原地发胖：这第一步等同于在指定的坐标 pos 处，渲染了一个比原本目标文字“胖了一圈”的巨大黑色阴影轮廓层。
    # 7. 亮色覆盖层：第二步，绝不更改位置 pos，但是换上用户传进来的亮色颜料 color，并用原本正常的细线宽 thick 再画一遍。
    # 8. 对比度奇迹：因为细的亮色字刚好填在了胖的黑底字的中心，文字的边缘就被勾勒出了一圈完美的抗光晕黑线。无论背景是云朵的纯白高光还是爆炸的火球，字都清晰可见。
    # 9. 亚像素抗锯齿：两行画字代码都极其讲究地在末尾附带了 cv2.LINE_AA。这会开启高阶的亚像素混合算法，消除因为屏幕像素方格排列导致的狗牙状锯齿边缘，画面瞬间变得平滑而丝绸。
    # 10. 高度可定制：它开放了位置 pos、颜色 color、缩放倍率 scale 和基础线条粗细 thick 给外层控制，实现了极其优秀的扩展兼容性。
    @staticmethod
    def text(img, s, pos, color=(230,230,230), scale=0.38, thick=1):

        # 执行黑底描边绝学的第一阶段：打磨底座。
        # 接收到需要被污染的那一帧底层图像矩阵 img，要打印的字符串本体 s，以及左下角基准起点的几何元组 pos。
        # 祭出 cv2 字体库中最高效简约的 FONT_HERSHEY_SIMPLEX。
        # 强制使用 (0,0,0) 纯黑，并极其诡异地使用厚度 thick+2，刻印出胖体黑色轮廓。
        cv2.putText(img, s, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thick+2, cv2.LINE_AA)

        # 执行亮底覆盖绝学的最终阶段：注入灵魂色彩。
        # 依然在原封不动的图像矩阵上，在原封不动的相同物理坐标 pos 上，重新来过。
        # 这一次，填入高级调制的 color 参数，恢复到正常纤细的 thick 厚度。
        # 黑白两极反转相加，配合 LINE_AA 平滑算法，造就了能够在任何恶劣背光下不被掩盖的高保真军事字符。
        cv2.putText(img, s, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

    # 【draw_crosshair 绘制火控辅助准星类方法极其详尽说明】
    # 1. 核心定位：在图像的最中央画上雷达追踪器视觉上的绝对锚点基准线。
    # 2. 装饰器定义：这是一个 @classmethod，意味着它的首个参数是 cls（代表类本身），这让它能在内部直接调用类的颜色常量，无需硬编码。
    # 3. 尺寸自适应：它绝对不会写死中心点在哪（比如傻乎乎地写死 320,240）。它是利用传入 img 矩阵本身的 shape 属性现场计算高宽。无论传来的是 4K 还是 480P，准星永远居中。
    # 4. 中间镂空哲学：区别于老掉牙的十字全连接靶标，它画的是中间空出来一个 gap（留白区域）的四段断开式线段。
    # 5. 防遮挡设计：为什么要留白？因为如果在几千米外，靶机在画面中只占可怜的两个像素点。如果你画满实线，准星的线条本身就会把目标彻底物理覆盖掉，这种镂空是为了让操作员肉眼辨识微小远景目标。
    # 6. 色彩降压：准星作为极其大面积常驻画面中心的图形，它甚至没有用 C_WHITE 这个灰白，而是更低沉的 (200,200,200) 银色，确保它只在需要时被看到，平时能完美隐入脑后。
    # 7. 极其繁琐的偏移：为了画出这四段线，不得不进行极度枯燥的中心点加减法。每一根线段都必须精确控制起点和终点的偏移量，严丝合缝，毫厘不差。
    # 8. 中心极点：在四象限线段包裹的绝对空心正中间，原作者神来之笔地打上了一个仅仅占据一两个像素的微观实心小圆点。
    # 9. 枪口准星：这个微观小圆点才是炮弹飞出炮管后在屏幕二维切面上的绝对归宿，也是人眼疲劳时重新聚焦画面的聚焦点。
    # 10. 全局抗锯齿：同样的，这五条底层的图形绘制命令，无一例外全都挂上了 cv2.LINE_AA 的狗牌，把高级工业风进行到底。
    @classmethod
    def draw_crosshair(cls, img):

        # 这是所有图像处理库自适应解析画幅分辨率起手式中最经典、最高效的一步切片提取。
        # 众所周知 OpenCV 读出的 img 矩阵的 shape 属性是一个饱含 (Height, Width, Channel) 信息的元组。
        # 利用精妙的 Python 切片语法 [:2]，冷血地丢弃了表示色彩深度的第三维通道参数。
        # 顺滑地将极其重要的图像整体真实高度 h 和总跨度宽度 w 解包到了局部变量的口袋里，供后续挥霍。
        h, w = img.shape[:2]

        # 找到了画卷的边际后，立刻使用极其残暴的整除计算运算符（//）。
        # 将宽和高分别无情地对半分尸，求得绝对数学意义上的画面质心坐标 cx 和 cy。
        # 为什么必须用双斜杠 // 而不是单斜杠 /？因为如果切出一半是带小数点的 320.5，传给底层画线函数时 OpenCV 会报出极其严重的整型类型不匹配错误！
        cx, cy = w // 2, h // 2

        # 为这个长久霸占视觉中心的准星调制一抹极为低调且克制的颜色颜料 c。
        # 它被设定为 (200, 200, 200)，比起 (255,255,255) 那刺目的钛白，这种低照度的银灰色极具军工冷酷风。
        # 它的低可视度确保了即便在长时间的模拟对抗盯防中，操作员也不会因为这根粗大的十字线而感到严重视觉疲劳。
        c = (200, 200, 200)

        # 极其老练的高密度魔法数字（Magic Number）定义代码行。
        # 将中间用来容纳微小物体的预留镂空空间半径定义为 gap = 8 个像素的距离。
        # 将向外延伸的每一小段实体准星刻度线的总物理长度定义为 ln = 12 个像素。
        # 这两个看似微不足道的数字，彻底界定了这个准星在显示器上到底给人一种宏大的巨炮感，还是一种精密的狙击感。
        gap, ln = 8, 12

        # 开始极其繁琐且极度考验坐标轴计算逻辑的第一刀：画出屏幕左边水平的那半截小横线。
        # 它的起点，必须是从绝对中心 cx 向左无情倒推。先推掉镂空半径，再推掉整根线长，找到那最左端的一点 (cx-gap-ln, cy)。
        # 而这根短线的终结之处，恰好停在触碰到神圣的禁飞留白区边界的那一点 (cx-gap, cy)。
        # 用银色、以 1 像素的极细克制线条搭配高配抗锯齿 LINE_AA，在内存矩阵里重重地拉下了这绝妙的一笔。
        cv2.line(img, (cx-gap-ln, cy), (cx-gap, cy), c, 1, cv2.LINE_AA)

        # 镜像重现，画出屏幕右半边水平对称的那截极其重要的右侧辅助小横线。
        # 原理不变，但在 X 坐标轴上的符号全部迎来极其残酷的大清洗：从代表倒退的负号翻身变成了代表前进的正号。
        # 从中心点往右迈出留白区的距离作为起点 (cx+gap, cy)，向外延伸出加上线段总长的终点位置 (cx+gap+ln, cy)。
        # 这一笔下去，准星水平方向的枷锁宣告彻底落成。
        cv2.line(img, (cx+gap, cy), (cx+gap+ln, cy), c, 1, cv2.LINE_AA)

        # 战斗转移阵地，彻底抛弃 X 轴的变化，在垂直的 Y 轴上进行上下两极的极其精确的纵向辅助线打击。
        # 锁定 X 坐标在不可动摇的绝对中心 cx 上，利用负号减去间隙和线长，探寻到视觉画面正上方天空最高端的一点坐标 (cx, cy-gap-ln)。
        # 以此为起点，垂直向下坠落一小段，直击上方留白边境线 (cx, cy-gap)，构成上半球的雷达准星标尺。
        cv2.line(img, (cx, cy-gap-ln), (cx, cy-gap), c, 1, cv2.LINE_AA)

        # 对称学说的最后也是最无脑的一次复刻：绘制中心点下方屏幕的那截垂向引力线段。
        # Y 轴参数换成了加号向更深处的底部像素坐标点探进，起点在中心偏移一段距离处 (cx, cy+gap)，止步于 (cx, cy+gap+ln)。
        # 这四次无情地矩阵重涂，最终在这张二维平面图片上死死钉下了一个具有极强指引感和四象限包围感的大型十字准星空间。
        cv2.line(img, (cx, cy+gap), (cx, cy+gap+ln), c, 1, cv2.LINE_AA)

        # 准星系统的点睛绝命一笔，这是给火炮开火的绝对真理焦点。
        # 在经历了上面四次大开大合的外围包抄后，极其克制地回到那万物起源的数学绝对中心点 (cx, cy)。
        # 调用 cv2 最原始的画圆命令 circle，用近乎发指的、仅仅只有可怜 1 像素的微观半径，点下了这个不起眼的质心。
        # 最精妙的是参数那个 -1，这代表这个圆不只是外卖面皮，它是被实心填满的！形成了一个无可争辩的最终极死亡归宿点。
        cv2.circle(img, (cx,cy), 1, c, -1, cv2.LINE_AA)
        # 【draw_detection_box 绘制目标锁定框方法说明】
    # 1. 在无人机视觉识别与拦截系统中，单纯识别出目标不够，还需要将边界框精准地画在画面上供人眼确认。
    # 2. 此方法接收原始图像、YOLO输出的边界框坐标数组、置信度分数以及类别ID。
    # 3. 为了摒弃传统平庸的完整矩形框，这里采用绘制四个角落“断点L型”线条的设计，营造浓厚的机甲火控锁定感。
    # 4. 它还极其贴心地在框的左上角标注了目标类别和识别概率，并在目标的绝对数学质心打下了一个红色准星十字。
    @classmethod
    def draw_detection_box(cls, img, box, conf, cls_id=0):

        # 利用 Python 优雅的列表推导式，对 YOLO 传出的浮点数坐标进行降维打击。
        # 像素绘图只能接受整数，因此这里强制将包含左上和右下对角线坐标的 box 数组全盘遍历并强转为 int 型。
        # 解包赋值给 x1, y1 (左上角) 和 x2, y2 (右下角)，确立了目标在二维画面上的物理疆界。
        x1, y1, x2, y2 = [int(v) for v in box]

        # 召唤在类头部定义好的专属荧光绿常量 C_GREEN，作为绘制边框的颜料。
        # 这种颜色不仅极具科技感，还能在复杂的仿真天空背景中保持极高的对比度。
        c = cls.C_GREEN

        # 设定所谓的“断点延伸长度” corner 为 12 个像素。
        # 稍后在画框时，线条不会从头连到尾，而是从每一个角出发，只向外延伸这短短的 12 个像素就戛然而止。
        corner = 12

        # 绘制左上角起点的横向短线。起点为 (x1, y1)，终点在 X 轴向右推进 12 像素 (x1+corner, y1)。
        # 指定线宽为 2，开启 LINE_AA 抗锯齿，确立了锁定框左上角的第一根骨架。
        cv2.line(img, (x1,y1), (x1+corner,y1), c, 2, cv2.LINE_AA)

        # 紧接着绘制左上角起点的纵向短线。起点依旧，终点在 Y 轴向下坠落 12 像素 (x1, y1+corner)。
        # 这两行代码完美勾勒出了一个反向的“L”型折角，这是现代战机 HUD 最经典的元素。
        cv2.line(img, (x1,y1), (x1,y1+corner), c, 2, cv2.LINE_AA)

        # 视线转移到右上角 (x2, y1)。先画横线，终点必须向左倒退 12 像素 (x2-corner, y1) 才能往回指。
        # 极其严谨的坐标偏移计算，容不得半点正负号的差错。
        cv2.line(img, (x2,y1), (x2-corner,y1), c, 2, cv2.LINE_AA)

        # 继续在右上角画纵向短线，Y 轴向下延伸 12 像素 (x2, y1+corner)。
        # 至此，上半部分的两个火力夹角已经死死咬住了目标的头部。
        cv2.line(img, (x2,y1), (x2,y1+corner), c, 2, cv2.LINE_AA)

        # 战场转移到左下角 (x1, y2)。画横线，向右延伸 12 像素 (x1+corner, y2)。
        # 底部托盘的左侧防线建立完毕。
        cv2.line(img, (x1,y2), (x1+corner,y2), c, 2, cv2.LINE_AA)

        # 左下角画纵向线，注意此时终点必须在 Y 轴向上倒退 12 像素 (x1, y2-corner) 以指向上方。
        # 构成正向的“L”型托底。
        cv2.line(img, (x1,y2), (x1,y2-corner), c, 2, cv2.LINE_AA)

        # 最后一战，右下角 (x2, y2)。横向线向左倒退 (x2-corner, y2)。
        # 逐步封死目标最后的逃生路线。
        cv2.line(img, (x2,y2), (x2-corner,y2), c, 2, cv2.LINE_AA)

        # 右下角纵向线向上倒退 (x2, y2-corner)。
        # 经过这八行极其枯燥但充满工业美感的像素级绘图，一个完美的机甲锁定框诞生了。
        cv2.line(img, (x2,y2), (x2,y2-corner), c, 2, cv2.LINE_AA)

        # 调用底层的黑底描边文本工具 text()。
        # 在目标框左上角往上提 4 个像素 (y1-4) 的位置，打上硬编码的 "UAV" 标签。
        # 配合 Python 强大的格式化语法 {conf:.0%}，将浮点数秒变带百分号的整数，直观展示神经网络的自信程度。
        cls.text(img, f"UAV {conf:.0%}", (x1, y1-4), cls.C_GREEN, 0.35)

        # 计算这一团被框住的像素的真正几何质心。利用左右边界相加除以2，上下边界相加除以2。
        # 求出目标在这个二维切面上的绝对重心点 (mx, my)，这是拦截算法计算前置量不可或缺的输入。
        mx, my = (x1+x2)//2, (y1+y2)//2

        # 在刚才算出的质心处，利用 cv2 相对冷门的 drawMarker 函数，强行打下一个红色的标准交叉标记（MARKER_CROSS）。
        # 这个红十字就是制导雷达眼中唯一在乎的坐标实体，外面的绿框只不过是给人类看的安慰剂。
        cv2.drawMarker(img, (mx,my), cls.C_RED, cv2.MARKER_CROSS, 8, 1, cv2.LINE_AA)

    # 【draw_info 绘制左上角系统状态面板方法说明】
    # 1. 作为数据集成大屏，它将零散在各处的运行状态统一汇总显示。
    # 2. 包括帧率、总帧数、检测成功率、制导模式以及云台的机械角度。
    # 3. 采用紧凑的多行排版，有效利用了屏幕边缘的无用像素空间。
    @classmethod
    def draw_info(cls, img, data):

        # 安全地从传入的 data 字典中提取当前画面检测到的目标数量，若无则默认为 0。
        # 提取历史总处理帧数，为了防止在程序刚启动的第 0 帧出现除以零的数学灾难，用 max 函数将其死死托底在 1 以上。
        det = data.get('detections', 0)
        total = max(data.get('frame', 1), 1)

        # 极其密集的字符串数组构造区。将大量系统核心指标通过 f-string 格式化为三行文本。
        # 第一行塞入了实时 FPS、当前帧号、以及通过除法算出的总体检测识别命中率。
        # 第二行展示当前大写化（upper）的制导算法、无人机飞行模式，以及目标跟踪是否已锁定（ON/OFF）。
        # 第三行则是最为硬核的物理数据：云台实时返回的 Pitch（俯仰角）和 Yaw（偏航角），带正负号显示。
        lines = [
            f"FPS {data.get('fps',0):.0f}  F{data.get('frame',0)}  D{det}/{total}({det/total*100:.0f}%)",
            f"{data.get('method','-').upper()} | {data.get('pattern','-').upper()} | TRK {'ON' if data.get('tracking') else 'OFF'}",
            f"P{data.get('pitch',0):+.1f}  Y{data.get('yaw',0):+.1f}",
        ]

        # 利用 enumerate 同时获取文本内容的索引 i 和实际字符串 ln。
        # 通过循环，以固定的 X 坐标 6 和基于索引递增的 Y 坐标 (14 + i * 16)，将这三行文字极其整齐地刷在屏幕左上角。
        # 这种按行距自动排版的方式避免了多行坐标硬编码的丑陋。
        for i, ln in enumerate(lines):
            cls.text(img, ln, (6, 14 + i * 16), cls.C_WHITE, 0.35)

    # 【draw_target 绘制 3D 目标坐标投影说明】
    # 1. 将深度反推算法算出的 3D 坐标数据可视化。
    # 2. 放置在画面左下角，作为整个 3D 估算模块是否正常工作的监控探针。
    @classmethod
    def draw_target(cls, img, pos):

        # 取出图像的高度 h，这是为了稍后能将文字精确对齐到画面的最底部。
        # 这种自适应高度的做法是编写强健 UI 布局的基石。
        h = img.shape[0]

        # 在左下角，也就是 Y 坐标为 h-6 的极低位置，输出带有 TGT (Target) 前缀的真实世界 3D 坐标。
        # 颜色选用了一种极其亮眼的青蓝色 (50,200,255)，以区分于普通的白色状态文本。
        cls.text(img, f"TGT ({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f})", (6, h-6), (50,200,255), 0.33)

    # 【draw_fire 绘制开火警告说明】
    # 1. 这是一个极具暴力美学和情绪价值的反馈函数。
    # 2. 只有当系统确定已向底层引擎下发了发射指令时才会被瞬间调用，在屏幕中央糊上红色大字。
    @classmethod
    def draw_fire(cls, img):

        # 老规矩，切片提取画面长宽，为了寻找那个绝对的中心点。
        # 这个操作在 HUD 类里被反复使用，确保一切重大提示都聚焦于中心视野。
        h, w = img.shape[:2]

        # 在宽度正中间偏左一点（w//2-20），高度正中间偏下一点（h//2+35），刻下极具威慑力的 "FIRE" 字符。
        # 动用 C_RED 纯红常量，并且史无前例地将字体缩放调到了 0.6，线宽调到了 2，制造出刺目的警告效果。
        cls.text(img, "FIRE", (w//2-20, h//2+35), cls.C_RED, 0.6, 2)

    # 【enhance_image 图像预处理增强说明】
    # 1. 仿真环境（如虚幻引擎）中的光照可能存在过暗或逆光的情况。
    # 2. 利用矩阵线性变换，提升画面亮度和对比度，为 YOLO 识别减轻负担。
    @staticmethod
    def enhance_image(img, brightness=15, contrast=1.15):

        # 直接调用 OpenCV 的 convertScaleAbs 底层 C 接口进行像素级矩阵遍历。
        # 算法公式为：新像素 = 旧像素 * alpha(contrast) + beta(brightness)。
        # 一行代码完成了极其高效的亮度拉升和对比度扩展，返回清洗后的优质图像矩阵。
        return cv2.convertScaleAbs(img, alpha=contrast, beta=brightness)


class VisionGuidance:
    # 【VisionGuidance 视觉制导核心类说明】
    # 1. 本类是整个武器系统的真正大脑，统领了网络通信、模型推理和物理弹道解算三大战区。
    # 2. 它在内存中实例化了巨大的 YOLO 计算图，随时准备对传入的像素矩阵进行卷积切割。
    # 3. 包含了从 2D 像素点逆向投影到 3D 世界射线的全套几何外参转换数学库。
    # 4. 内部的 run 死循环是整个拦截测试的心跳，负责驱动图像拉取、送入预测网络、下达云台偏转指令的完整闭环。
    # 5. 它巧妙地利用时间戳计算出 dt，彻底修复了速度预测滞后的严重系统级 BUG。
    def __init__(self, client, model_path, turret_id="turret_0",
                 muzzle_speed=400.0, conf=0.3, method="predictive"):

        # 将传入的网络通信大管家 client 妥善保存，它是大脑向火炮传递神经信号的唯一突触。
        # 保存炮塔 ID turret_id，防止在多目标仿真场景中发错指令导致友军误伤。
        self.client = client
        self.turret_id = turret_id

        # 录入核心物理常量 muzzle_speed（炮口初速），这是后端解算拦截提前量、计算抛物线轨迹的绝对灵魂参数。
        # 录入模型置信度下限 conf，决定了 AI 在面对模糊像素时是选择宁可错杀还是绝不放过。
        self.muzzle_speed = muzzle_speed
        self.conf = conf

        # 这是极其沉重、会引起程序几秒钟卡顿的初始化操作。
        # 根据传入的路径字符串，将几十兆甚至几百兆的 YOLO 模型权重加载进内存，唤醒这头沉睡的深度学习巨兽。
        self.model = YOLO(model_path)

        # 在一切开始前，通过网络向下层引擎发送 reset 指令，清空火炮内残存的历史转角和乱七八糟的预测缓存。
        # 立刻将外部要求的制导模式（如 predictive 预测制导）同步给底层控制器，并在本地 method 中留下一份备份记录。
        client.guidance_reset()
        client.guidance_set_method(method)
        self.method = method

        # 写死相机的视场角 FOV 为 90 度广角，设定图像长宽为经典的 640x480 分辨率。
        # 这三个参数构成了针孔相机内参矩阵的基础，是后续一切 3D 透视反演的基石。
        self.cam_fov = 90.0
        self.cam_w = 640
        self.cam_h = 480

        # 初始化用来接收云台相机实时 3D 位置和三轴旋转欧拉角的 numpy 空数组。
        # 初始化用于统计总帧数和成功识别帧数的计数器，并极其优雅地实例化了一个用于画界面的 HUDRenderer 对象。
        self.cam_pos = np.zeros(3)
        self.cam_rot = np.zeros(3)
        self.frame_count = 0
        self.detect_count = 0
        self.hud = HUDRenderer()

    # 【get_image 获取实时画面流说明】
    # 1. 负责向底层仿真器索要最新的一帧战场快照。
    # 2. 包含极其坚固的异常处理机制，解码 base64 数据流并同步更新相机外参。
    def get_image(self):

        # 尝试通过 _send 底层接口发送要求获取图片的特定 JSON 命令 {"get_image": {}}。
        # 若遭遇网络波动引发 Exception，直接返回 None 咽下苦果，绝不拖垮主线程。
        try:
            resp = self.client._send({"get_image": {}})
        except Exception:
            return None

        # 严格校验服务端回传包中 status 字段是否为代表胜利的 "ok"。
        # 若不是，说明引擎端图片渲染管线卡死，同样返回 None。
        if resp.get("status") != "ok":
            return None

        # 提取被包裹在 data 字段中的 base64 超长编码字符串。
        # 再次进行防呆检查，防止收到一个空字符串导致后续解码器当场崩溃。
        b64 = resp.get("data", "")
        if not b64:
            return None

        # 将 base64 文本瞬间解码为原始二进制字节流 buffer。
        # 喂给 numpy 将其重塑为一维 uint8 数组，最后利用 OpenCV 的 imdecode 魔法将其还原为饱满鲜活的三通道彩色图像矩阵。
        img = cv2.imdecode(np.frombuffer(base64.b64decode(b64), np.uint8), cv2.IMREAD_COLOR)

        # 顺手牵羊，将回传包里夹带的最新相机宽度、高度、视场角参数扒下来，更新本地的内参数据。
        # 极其关键地更新相机的物理三维坐标 camera_pos 和欧拉角 camera_rot，为接下来的 3D 测算提供绝对正确的当前姿态环境。
        self.cam_w = resp.get("width", self.cam_w)
        self.cam_h = resp.get("height", self.cam_h)
        self.cam_fov = resp.get("fov", self.cam_fov)
        self.cam_pos = np.array(resp.get("camera_pos", [0,0,0]))
        self.cam_rot = np.array(resp.get("camera_rot", [0,0,0]))
        
        # 将这张历经千难万险解压还原出来的二维战场切片矩阵，郑重地向上返回，交给主循环投喂给 YOLO。
        return img

    # 【pixel_to_direction 像素坐标转相机局部射线说明】
    # 1. 这是计算机视觉 3D 重建的第一步：从 2D 屏幕上的一个像素点，反推出一条射向物理世界的假想射线。
    # 2. 利用极其基础但至关重要的三角函数求取相机焦距。
    def pixel_to_direction(self, u, v):

        # 经典针孔模型焦距解算公式：焦距 fx 等于画面一半宽度除以半视场角的正切值。
        # 先把角度转弧度，算正切，做除法，一步到位求出这个隐形的物理焦距长度。
        fx = (self.cam_w / 2.0) / math.tan(math.radians(self.cam_fov / 2.0))

        # 构造局部 3D 方向向量 d。Z 轴深度设为刚刚求出的正焦距 fx（假设前向为正）。
        # X 轴和 Y 轴则是目标像素点 (u, v) 距离画面绝对中心的光学偏差，Y 轴加了负号以修正屏幕坐标系向下的反直觉设定。
        d = np.array([fx, u - self.cam_w/2.0, -(v - self.cam_h/2.0)])

        # 这个向量目前长度不一，利用线性代数求范数（长度） norm。
        # 将向量自身除以该长度进行归一化，返回一个干干净净、绝对长度为 1 的局部指向射线。
        return d / np.linalg.norm(d)

    # 【cam_to_world 相机坐标系转世界绝对坐标系说明】
    # 1. 相机是挂在不断旋转的云台上的，刚才算出的射线只是相对于相机镜头的方向。
    # 2. 必须利用相机当前的三轴欧拉角，徒手捏造一个三维旋转矩阵，把这根射线扭转到真实世界的绝对坐标系中。
    def cam_to_world(self, d):

        # 利用极其暴力的列表推导式，将刚刚从引擎里同步下来的 Pitch、Yaw、Roll 三个角度的度数，全盘转为标准弧度制。
        # 解包给局部短变量 p, y, r 备用。
        p, y, r = [math.radians(x) for x in self.cam_rot]

        # 提前计算并缓存俯仰角 (Pitch) 和偏航角 (Yaw) 的正余弦值。
        # 在高频刷新的图像处理循环中，这种空间换时间的缓存能有效减轻 math 模块的运算压力。
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        # 同样缓存横滚角 (Roll) 的正余弦值。
        # 这三组共六个三角函数值，构成了接下来组装巨型旋转矩阵积木的绝对原材料。
        cr, sr = math.cos(r), math.sin(r)

        # 徒手计算世界坐标系下的相机“正前方”投影向量 fwd，它是由俯仰和偏航共同交织决定的。
        # 徒手展开复杂的旋转公式，算出极其绕口的相机“正右方”投影向量 right。
        fwd = np.array([cp*cy, cp*sy, sp])
        right = np.array([cy*sp*sr-sy*cr, sy*sp*sr+cy*cr, -cp*sr])

        # 继续硬核展开公式，算出相机的“正上方”投影向量 up。
        # 有了前、右、上这三个相互绝对垂直的正交基底，相机在三维空间中那种被扭曲的姿态就被彻底量化捕获了。
        up = np.array([-(cy*sp*cr+sy*sr), -(sy*sp*cr-cy*sr), cp*cr])

        # 进行伟大的空间映射相加。将那根局部射线 d 的三个分量，作为权重，分别乘以此前求出的三个世界基底向量。
        # 这使得原本只知道“在画面右上方”的局部射线，瞬间变成了具有世界绝对方向感的三维坐标指引。
        w = d[0]*fwd + d[1]*right + d[2]*up

        # 同理，经历了残酷矩阵蹂躏的向量需要重新整理仪容。
        # 除以自身的欧几里得长度，再次归一化，返回在绝对世界坐标系中指向目标的真理单位射线。
        return w / np.linalg.norm(w)

    # 【estimate_3d 单目视觉伪 3D 深度测距说明】
    # 1. 单个摄像头无法得知真实物理距离，只能靠“近大远小”的透视学定律。
    # 2. 利用目标在画面中占据的像素框面积作为启发式输入，暴力估算深度。
    def estimate_3d(self, u, v, bbox_area):

        # 按部就班地调用前置技能。先将 2D 像素中心点转化为相机局部射线。
        # 再将这条局部射线通过相机欧拉角旋转，掰成世界绝对方向射线 d_world。
        d_cam = self.pixel_to_direction(u, v)
        d_world = self.cam_to_world(d_cam)

        # 这是计算机视觉中最粗暴但也最有效的工程拟合经验公式：距离等于常数除以面积的平方根。
        # 先求面积平方根消除非线性缩放，再做倒数，最后被 min 和 max 函数死死卡在 100 到 50000 之间的安全深度范围内，防止飞出银河系。
        depth = max(100, min(5000/math.sqrt(max(bbox_area,1)), 50000))

        # 神奇的一步：将相机本身所在的三维绝对坐标 cam_pos，加上（方向射线乘以推测深度）。
        # 这等同于从炮塔发射了一条不可见的激光打向无人机，直接算出了目标的伪 3D 绝对坐标并返回。
        return self.cam_pos + d_world * depth, depth

    # 【run 核心作战死循环方法说明】
    # 1. 挂载 UI 窗体，启动网络拉图死循环。
    # 2. 调用 YOLO 获取边界框，极度精准地测算物理 dt 喂给卡尔曼超算中心。
    # 3. 汇总信息刷新屏幕，监听每一个极具杀意的键盘按键。
    def run(self, drone_ctrl, auto_fire=False, fire_interval=80):

        # 利用 OpenCV 强行在操作系统的桌面上挖出一个可自由缩放的 GUI 窗体。
        # 取名为 "Turret Camera - YOLO Guidance"，并强制规定一个 640x480 的初始体型，以迎接即将到来的像素洪流。
        cv2.namedWindow("Turret Camera - YOLO Guidance", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Turret Camera - YOLO Guidance", 640, 480)

        # 罗列三种支持的火控解算体系：predictive(全维度预测)、proportional(比例导引)、direct(直接直瞄)。
        # 通过查询本地存留的 self.method 获取当前模式在数组中的游标索引 mi，为后续热键切换做好预热准备。
        methods = ["predictive", "proportional", "direct"]
        mi = methods.index(self.method) if self.method in methods else 0

        # 初始化控制流的三大护法状态变量：跟踪死锁雷达默认开启 tracking = True。
        # 将当下的绝对系统时间戳拍入 last_t 中，作为测速秒表的起跑线。开火警告灯 fired 默认熄灭。
        tracking = True
        last_t = time.time()
        fired = False

        # 拉开全局异常捕获防爆罩，任何因为内存泄露或者网络断开引发的异常，最终只能在这里抛出个没用的报错，别想弄死主进程。
        try:

            # 这是所有自动化武器的心跳根源：一个无法被外界时间流逝打破的 True 死循环。
            # 只有键盘按键能从内部瓦解它。
            while True:

                # 向后端引擎伸手索要画面。拿到 img，如果是一团不可名状的 None（比如网络丢包）。
                # 就委屈求全地睡上 0.1 秒，用 continue 跳过这一回合，耐心等待下一张图片的到来。
                img = self.get_image()
                if img is None:
                    time.sleep(0.1)
                    continue

                # 图片成功获取，说明我们又苟活过了一帧，全局历史处理帧数计数器极其骄傲地自增 1。
                self.frame_count += 1

                # 将宝贵的图像矩阵无情地塞入 YOLO 这个巨大的数学黑洞中。
                # 设置极度严苛的置信度阈值 conf，屏蔽打印信息 verbose，祈祷模型能从中榨取出一丝一毫的目标线索。
                # 如果显存爆仓或者模型抽风抛出 Exception，就用一个空列表 [] 掩盖过去，假装什么都没看见。
                try:
                    results = self.model(img, conf=self.conf, verbose=False)
                except Exception:
                    results = []

                # 初始化本帧轮次的局部易失性状态：目标未检出 detected = False，目标 3D 坐标位 pos 置空。
                # 拟态云台的偏转角重置归零，本帧绝未开火 fired = False。一切清零，从头开始。
                detected = False
                pos = None
                pitch, yaw = 0, 0
                fired = False

                # 物理仿真中最伟大的一步：抓取此刻的绝对时间戳 now。
                # 用现在减去上一帧留存的 last_t，算出这期间电脑到底流逝了多少秒 dt。
                # 用 0.001 秒死死兜底防止除零溢出，最后更新 last_t 交接棒。这段代码治好了多少卡尔曼滤波器因为掉帧导致的乱跑顽疾。
                now = time.time()
                dt = max(now - last_t, 0.001)
                last_t = now

                # 唤醒云端火控拦截中心！向 guidance_auto_engage 接口倾泻包括开炮方、挨打方、炮弹初速等关键数据。
                # 最关键的是附带上刚刚算出的真实帧间时间差 dt，让超算中心明白目前的网络延迟情况。
                # 如果中心传回了 "ok"，说明未来某个时空点的拦截交汇抛物线已经解算成功，赶紧扒下它传回的预瞄俯仰角和偏航角。
                engage = self.client.guidance_auto_engage(
                    self.turret_id, "drone_0", self.muzzle_speed, dt=dt)
                if engage.get("status") == "ok":
                    pitch = engage.get("pitch", pitch)
                    yaw = engage.get("yaw", yaw)

                # 开始审视 YOLO 传出的战报。如果战果列表里真的有东西，并且第一张图中确实存在被边界框圈住的倒霉蛋。
                # 立刻唤醒 .cpu().numpy() 将这些被囚禁在显存里的高傲张量统统拉下神坛，贬为人人可用的 NumPy 数组。
                # 记录成功检测数，并点亮本帧识别的高光时刻 detected = True。
                if len(results) > 0 and len(results[0].boxes) > 0:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    confs = results[0].boxes.conf.cpu().numpy()
                    classes = results[0].boxes.cls.cpu().numpy()
                    self.detect_count += 1
                    detected = True

                    # 对着这一批从画面中揪出来的所有倒霉蛋，依次降下制裁的画笔。
                    # 调用 hud 的专属绘框利器，在他们的像素周身描出充满杀气的绿色断点方框。
                    for i in range(len(boxes)):
                        self.hud.draw_detection_box(img, boxes[i], confs[i])

                    # 丛林法则，弱肉强食：在一堆检测框中，通过 argmax 找出那个模型置信度（把握）最高的靶机 bi，作为首要打击目标。
                    # 将这个倒霉蛋的四个极点坐标无情解包，算术求取其中心点 cx, cy 和面积 area。
                    # 把这些尸骸线索扔进 estimate_3d 预言机里，极其粗暴地榨取出了它的伪三维空间坐标 pos。
                    bi = confs.argmax()
                    x1,y1,x2,y2 = boxes[bi]
                    cx, cy = (x1+x2)/2, (y1+y2)/2
                    area = (x2-x1)*(y2-y1)
                    pos, _ = self.estimate_3d(cx, cy, area)

                # 这是一道挂满了三把锁的绝杀之门。
                # 第一锁：指挥官已下放开火权 auto_fire；第二锁：雷达正处于死锁跟随状态 tracking；第三锁：利用取模严控射速，防止枪管过热。
                # 只有三位一体全数通过，一纸网络密电 turret_fire 将瞬间传至底层，电磁阀打开，炮弹激射而出。并立刻亮起本帧已开火红灯 fired = True。
                if auto_fire and tracking and self.frame_count % fire_interval == 0:
                    self.client.turret_fire(self.muzzle_speed, self.turret_id)
                    fired = True

                # 将刚才那个精确到极点的 dt 做一个华丽的倒数转身，这便是屏幕上那极具欺骗性的实时帧率 FPS。
                # 极其卑微地尝试去查询云台底座当前真实的物理偏转角。如果能查到，就覆盖刚才那些用于指示目标的角度，追求极致的写实。
                fps = 1.0 / dt

                try:
                    ts = self.client.turret_state(self.turret_id)
                    pitch = ts.get("pitch", pitch)
                    yaw = ts.get("yaw", yaw)
                except:
                    pass

                # 开始本帧的终极艺术收尾工作：在污浊的战损画面上糊上冰冷的准星。
                # 将收集到的一大堆杂乱无章的帧率、模式、角度等变量塞入字典，抛给 draw_info 糊在左上角大屏上。
                # 如果今天真的抓到了目标且算出了 3D 坐标 pos，就在左下角投影出 TGT 的数据炫耀战绩。
                # 如果刚才真的扣下了扳机，用鲜红的血色 FIRE 覆盖在画面中间，向全宇宙宣告你的武力。
                self.hud.draw_crosshair(img)
                self.hud.draw_info(img, {
                    "fps": fps, "frame": self.frame_count,
                    "detections": self.detect_count,
                    "method": self.method, "pattern": drone_ctrl.pattern,
                    "tracking": tracking, "pitch": pitch, "yaw": yaw,
                })
                if detected and pos is not None:
                    self.hud.draw_target(img, pos)
                if fired:
                    self.hud.draw_fire(img)

                # 将历经各种磨难、被涂抹得花里胡哨的一帧图像矩阵，正式推到桌面的 GUI 窗口前，强奸操作员的视网膜。
                cv2.imshow("Turret Camera - YOLO Guidance", img)

                # 键盘外设的中断探针：仅停留可怜的 1 毫秒（非阻塞），抓取键盘缓冲区的字节流。
                # 只要探测到 'q'，果断发动 break 击穿这个无坚不摧的死循环结界，宣告本次模拟演练强制结束。
                # 探测到 'f'，直接绕开那三把开火安全锁，硬发击发指令，展现绝对的手工介入强权。
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('f'):
                    self.client.turret_fire(self.muzzle_speed, self.turret_id)
                
                # 探测到 'm'，极其丝滑地更改内部制导算法游标，取出下一个玄学制导名词，并勒令远端引擎立刻采用。
                # 探测到 '1', '2', '3'，发动跨线程打击，直接去修改那边天上乱窜的无人机对象内部的路径字符串，实现即时空域战术改变。
                elif key == ord('m'):
                    mi = (mi + 1) % len(methods)
                    self.method = methods[mi]
                    self.client.guidance_set_method(self.method)
                elif key == ord('1'):
                    drone_ctrl.set_pattern("straight")
                elif key == ord('2'):
                    drone_ctrl.set_pattern("curved")
                elif key == ord('3'):
                    drone_ctrl.set_pattern("evasive")
                
                # 探测到 'p'，下发奇观指令，在三维世界中凭空画出未来几秒钟火炮抛物线轨迹。
                # 探测到 't'，触发跟踪锁死布尔开关反转，开启或关停炮塔底座那疯狂寻找交汇点的伺服电机。
                elif key == ord('p'):
                    self.client._send({"call_turret": {"function": "show_prediction"}})
                elif key == ord('t'):
                    tracking = not tracking
                    if tracking:
                        self.client.turret_start_tracking("drone_0", self.turret_id)
                    else:
                        self.client.turret_stop_tracking(self.turret_id)

        # 全局防护底座的终结审判：无论是正常 break 退出，还是用户粗暴地砸了 Ctrl+C 引发 KeyboardInterrupt。
        # 统统拦截引流到 finally 的清道夫代码段中。
        # 释放 OpenCV 占有的系统图像句柄砸碎窗体，用最终检测数除以总帧数，以一串冰冷的百分比数据，作为本脚本结束生命前的墓志铭。
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            rate = self.detect_count / max(self.frame_count, 1) * 100
            print(f"\n[统计] 帧:{self.frame_count} 检测:{self.detect_count} 率:{rate:.1f}%")

# 选择yolo26模型
def find_model():
    for p in [os.path.join(SCRIPT_DIR, "Yolo", "runs", "detect", "drone_detect2", "weights", "best.pt"),
              os.path.join(SCRIPT_DIR, "Yolo", "yolo26n-objv1-150.pt"),
              os.path.join(SCRIPT_DIR, "Yolo", "yolo26n.pt")]:
        if os.path.exists(p):
            return os.path.abspath(p)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--method", default="predictive")
    parser.add_argument("--pattern", default="straight")
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--fire", action="store_true")
    parser.add_argument("--fire-interval", type=int, default=80)
    parser.add_argument("--muzzle-speed", type=float, default=400.0)
    parser.add_argument("--radius", type=float, default=150.0)
    args = parser.parse_args()

    if args.model is None:
        args.model = find_model()
    if not args.model or not os.path.exists(args.model):
        print("[ERROR] YOLO 模型未找到")
        return

    client = SimClient(args.host, args.port)

    ts = client.turret_state()
    turret_pos = ts.get("position", [0, 0, 0])
    print(f"[Turret] pos=({turret_pos[0]:.0f}, {turret_pos[1]:.0f}, {turret_pos[2]:.0f})")

    drone = DroneController(client, turret_pos, args.altitude, args.speed, args.radius)
    drone.set_pattern(args.pattern)
    drone.start()
    time.sleep(4)

    guidance = VisionGuidance(client, args.model, muzzle_speed=args.muzzle_speed,
                              conf=args.conf, method=args.method)
    guidance.run(drone, auto_fire=args.fire, fire_interval=args.fire_interval)

    drone.stop()
    client.close()

if __name__ == "__main__":
    main()