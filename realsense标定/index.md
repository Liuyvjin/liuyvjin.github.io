# RealSense 标定

Python 实现的 RealSense 相机类，以及相机标定类。
<!--more-->

# RealSense 类

```python
import pyrealsense2 as rs
import numpy as np
import cv2

class Camera(object):
    def __init__(self, width=640, height=480):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
        self.pipeline_profile = self.pipeline.start(self.config)
        self.device = self.pipeline_profile.get_device()
        # 深度图后处理
        self.colorizer = rs.colorizer()
        self.post_process = True
        self.filters = [rs.spatial_filter(),
                        rs.temporal_filter(),]

        # 深度图对齐到彩色图
        self.align = rs.align(rs.stream.color)

        # cam init
        print('Camera init ...')
        for i in range(50):
            frames = self.pipeline.wait_for_frames()
        self.mtx = self.getIntrinsics()
        print('Camera init done.')

    def get_data(self):
        while True:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            # post-process
            if self.post_process:
                for f in self.filters:
                    depth_frame = f.process(depth_frame)

            if not depth_frame or not color_frame:
                continue

            depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            break
        return color_image, depth_image, depth_colormap

    def getIntrinsics(self):
        """ 获取内参 """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()
        mtx = [intrinsics.width, intrinsics.height, intrinsics.ppx, intrinsics.ppy, intrinsics.fx, intrinsics.fy]
        camIntrinsics = np.array([[mtx[4],0,mtx[2]],
                                  [0,mtx[5],mtx[3]],
                                 [0,0,1.]])
        return camIntrinsics

    def __del__(self):
        self.pipeline.stop()


def vis_pc(pc):
    import open3d as o3d
    pc1 = o3d.geometry.PointCloud()
    pc1.points = o3d.utility.Vector3dVector(pc)
    o3d.visualization.draw_geometries([pc1])

if __name__ == "__main__":
    cam = Camera(width=640, height=480)
    depth_sensor = cam.pipeline_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth scale is: ", depth_scale)

    while True:
        color, depth, depth_colormap = cam.get_data( )

        cv2.namedWindow('depth_colormap')
        cv2.imshow('depth_colormap', depth_colormap)
        cv2.namedWindow('color')
        cv2.imshow('color', color)
        c = cv2.waitKey(1)
        if c == 27: # 按下esc退出
            exit()
        if c == ord('f'):
            cam.post_process ^= True
            print("Post Process ", "On" if cam.post_process else "OFF")
```
# 标定
```python
from camera import Camera
import numpy as np
import cv2
# import matplotlib.pyplot as plt

def rtvec2transform(rvec, tvec):
    """ 将旋转向量和位移向量, 转换为矩阵形式

    :param rvec: 旋转向量 (3,1) 或 (1,3)
    :param tvec: 位移向量 (3,1) 或 (1,3)
    :return: 齐次变换矩阵 (4, 4)
    """
    rmat, _ = cv2.Rodrigues(rvec)
    trans = np.identity(4)
    trans[0:3, 0:3] = rmat
    trans[0:3, 3] = tvec.squeeze()
    return trans


class Calibrater:
    def __init__(self, get_img, pattern_shape=(8, 6), pattern_size=0.15):
        """
        :param get_img: 从相机获取图片的函数
        :param pattern_shape: 标定板角点数, defaults to (8, 6)
        :param pattern_size: 标定板格点尺寸(m), defaults to 0.15
        :param img_num: 标定内参时需要采集的有效标定板图片数量
        """
        self.get_img = get_img
        # 准备角点真实坐标
        self.pat_w, self.pat_h = pattern_shape[0], pattern_shape[1]
        self.obj_p = np.zeros((self.pat_w * self.pat_h, 3), np.float32)  # [m*n, 3]
        self.obj_p[:, :2] = pattern_size * np.mgrid[0:self.pat_w, 0:self.pat_h].T.reshape(-1, 2)
        # 相机像素
        img = get_img()
        self.h, self.w = img.shape[:2]
        print("Img shape: ({}, {})".format(self.w, self.h))
        # 迭代终止条件
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 0.001)

    def calibrate(self, img_num, save=False):
        """自动采集 self.img_num 张图片, 标定 mtx 和 dist, 并进行重投影评估
        :param img_num: 拍摄图片数量
        :param save: 是否保存内参 mtx 和 dist
        """
        img_points = []
        obj_points = []
        cv2.namedWindow('calibrate', cv2.WINDOW_AUTOSIZE) # 实时显示
        print("Press Enter to capture image")
        cnt = 0
        while cnt < img_num:  # 共需要记录 self.img_num 组数据
            img = self.get_img()
            ret, corners = self.find_corners(img)
            if ret == True:
                cv2.drawChessboardCorners(img, (self.pat_w, self.pat_h), corners, ret)  # 绘图

            cv2.imshow("calibrate", img)
            c = cv2.waitKey(5)
            if c == 13 and ret == True:  # 按下 enter 捕获当前图像角点
                img_points.append(corners)
                obj_points.append(self.obj_p)
                print("captured {} / {} images".format(cnt := cnt+1, img_num) )
            elif c == 27 or c == ord('q'):
                exit()
        cv2.destroyAllWindows()
        # 标定
        ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (self.w, self.h), None, None)
        # 评估标定结果
        self.eval_reproj_error(img_points, rvecs, tvecs, self.mtx, self.dist)
        print("mtx: \n", self.mtx)  # 内参矩阵
        print("dist: \n", self.dist)  # 畸变矩阵
        if save:
            np.savez("intrinsics.npz", mtx=self.mtx, dist=self.dist)

    def find_corners(self, img, subpix=True):
        """查找图像中的角点, 若找到返回 ret == True

        :param img: BGR 三通道彩色图像
        :param subpix: 是否进行精细化搜索
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.pat_w, self.pat_h), None, cv2.CALIB_CB_ADAPTIVE_THRESH)
        if ret == True and subpix:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), self.criteria) # 精细化搜索
        return ret, corners

    def eval_reproj_error(self, img_points, rvecs, tvecs, mtx, dist):
        """计算重投影误差

        :param img_points: 角点像素坐标 (img_num, n*m, 2)
        :param rvecs: 旋转向量 (img_num, 3)
        :param tvecs: 位移向量 (img_num, 3)
        """
        mean_error = 0
        for i in range(len(rvecs)):
            reproj_p, _ = cv2.projectPoints(self.obj_p, rvecs[i], tvecs[i], mtx, dist)
            mean_error += cv2.norm(reproj_p, img_points[i], cv2.NORM_L2) / len(reproj_p)
        print( "mean reprojection error: {}".format(mean_error / len(img_points)) )

    def draw_axis(self, img, subpix=False):
        """实时绘制三维坐标
        :param img: BGR 彩色图像
        :param subpix: 是否进行精细化搜索, 开启会比较卡
        """
        axis_3d = np.float32([[0.5,0,0], [0,0.5,0], [0,0,-0.5]])  # 要绘制的 3D 点
        ret, corners = self.find_corners(img, subpix)
        if ret==True:
            ret, rvec, tvec = cv2.solvePnP(self.obj_p, corners, self.mtx, self.dist)
            # 将3D点投影到图像平面
            axis_2d, _ = cv2.projectPoints(axis_3d, rvec, tvec, self.mtx, self.dist)
            axis_2d = axis_2d.astype(int).squeeze()
            origin = corners[0].astype(int).ravel()
            img = cv2.line(img, origin, axis_2d[0], (255,0,0), 5)
            img = cv2.line(img, origin, axis_2d[1], (0,255,0), 5)
            img = cv2.line(img, origin, axis_2d[2], (0,0,255), 5)
        return img


if __name__ == '__main__':
    cam = Camera()
    cali = Calibrater(lambda: cam.get_data()[0])
    cali.calibrate(img_num=5, save=True)

    with np.load('intrinsics.npz') as file:
        mtx, dist = [file[i] for i in ('mtx','dist')]
    print(mtx)
    print(dist)
    # 实时显示 3d 坐标
    cv2.namedWindow('3d_axis', cv2.WINDOW_AUTOSIZE)
    while True:
        img = cam.get_data()[0]
        img = cali.draw_axis(img)
        cv2.imshow('3d_axis', img)
        c = cv2.waitKey(1)
        if c == 27 or c == ord('q'):
            exit()

```
<!--more-->

