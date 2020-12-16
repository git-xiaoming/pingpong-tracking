# pingpong-tracking乒乓球轨迹跟踪
## 前言
这是我大一年度立项写的基于*OpenCV*轨迹追踪代码，大一知识面窄，对 *C++* 也只懂皮毛，参考了许多网上的代码。最近入门 *github* ，就拿这仅有的一点代码积累当练手吧。
## 项目简介
项目最初计划实现乒乓球陪练机器人，我负责视觉模块，主要目标是获取每一帧运动球体的世界坐标。
## 模块思路
首先调用摄像头逐帧捕捉，每一帧中基于颜色分割乒乓球区域，再经过平滑、形态学操作初步降噪后得到包含完整乒乓球区域的二值图，提取轮廓，基于圆度、大小筛选出正确乒乓球轮廓，求取球心坐标。
调用双目相机，则可获得两个不同视角下乒乓球同一空间位置的图像坐标，依据对极几何原理，代入事先标定好的相机内外参，即可解算出这一帧乒乓球的世界坐标。
在连续帧流的处理中，若当前帧已检测到乒乓球，下一帧只搜索该坐标附近局域，以提高处理效率。
## 代码实现
- **thresh，track，RGB_thresh**三个工程主要用于每次试验前的参数范围确定（由于没有提供固定的实验室，只能去学校乒乓球场试验，每次都要重新安装摄像头，球的大小以及颜色参数都需重新确定范围）。
- **trial**工程用于计算空间两点的距离并与实际值对比，以验证三维坐标定位算法的可靠性。
- **capture**工程用于调用双目摄像头拍照，用于后续标定。
- **stereo_calib**工程用于双目相机标定，输出相机内参和双目相机间的旋转、平移矢量（*基于官方示例，有所修改*）。
- **camera2world**工程用于将乒乓球世界坐标的参考系转换到乒乓球桌一角（*x，y轴沿球桌边缘，z轴竖直向上*），通过在球桌上摆拍标定板结合*OpenCV*的**solvePnP**算法实现。
- **main**工程是主要算法，用于驱动双目相机逐帧捕捉乒乓球图像，并实时输出世界坐标，最终得到一系列空间点坐标，理想情况下拟合绘制在空间坐标系中近似为抛物线（*乒乓球运动轨迹*）。
## 结语
由于其他队友的轨迹预测模块遇到瓶颈，最终目标未能达成，结题展示有所缩水，或许是大一眼高手低了一些吧。。
首次体验*github*，搬上的代码水准不高，见谅！
