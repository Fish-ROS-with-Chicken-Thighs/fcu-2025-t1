import cv2
import numpy as np
import rclpy

# 工具类
class CVTools:
    def __init__(self, node):
        self.node = node
    #---------------------------------------------
    #--------------------基本类--------------------
    #---------------------------------------------

    # 获取视频信息
    @staticmethod
    def get_video_info(capture):
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = capture.get(cv2.CAP_PROP_FPS)

        out = cv2.VideoWriter('output_video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height)) # 创建VideoWriter对象

        return width, height, fps, out

    # 展示&保存视频
    @staticmethod
    def imshow_and_save(frame, out):
        windows_name: str = 'Camera 720p'
        cv2.namedWindow(windows_name, cv2.WINDOW_NORMAL) #窗口大小任意调节
        cv2.imshow(windows_name, frame)
        out.write(frame) # 写入视频

    # 标记坐标
    @staticmethod
    def mark(contour, frame_copy, originX, originY):
        x, y, w, h = cv2.boundingRect(contour)
        # 画外接矩形
        cv2.rectangle(frame_copy, (originX+x-5, originY+y-5), (originX+x+w+5, originY+y+h+5), (0, 255, 0), 2)
        # 找到图形轮廓中心坐标
        M = cv2.moments(contour)
        center_x = int(M['m10'] / M['m00'])
        center_y = int(M['m01'] / M['m00'])
        cv2.circle(frame_copy, (originX+center_x, originY+center_y), 1, (255, 0, 255), 1) #圆心
        cv2.putText(frame_copy, "[" + str(originX+center_x) + "," + str(originY+center_y) + "]", (originX+center_x, originY+center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1) #坐标
        return frame_copy
        
    # 逆光补偿
    @staticmethod
    def backlight_compensation(frame):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))
        result = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return result

    # 质心法轮廓去重
    @staticmethod
    def filter_contours_by_centroid(contours, min_dist=20):
        contour_centers = []
        filtered_contours = []

        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # 检查是否与已有质心距离过近
                if all(np.hypot(cx - x, cy - y) > min_dist for x, y in contour_centers):
                    contour_centers.append((cx, cy))
                    filtered_contours.append(cnt)  # 仅保留间距足够的轮廓

        return filtered_contours

    # 质心法霍夫圆去重
    @staticmethod
    def filter_best_circle(circles):
        if circles is None:
            return None
        circles = np.uint16(np.around(circles))

        # 假设每个圆的得分为其半径（可以根据实际情况调整）
        # 这里假设半径越大，得分越高
        circles_with_scores = [(x, y, r, r) for x, y, r in circles[0, :]]

        # 按圆心 (x, y) 进行聚类，并选择得分最高的圆
        filtered = []
        for c in circles_with_scores:
            x, y, r, score = c
            # 如果该圆和已有的圆中心相差很近，就比较得分
            found = False
            for i, (fx, fy, fr, fscore) in enumerate(filtered):
                if np.linalg.norm(np.array([x, y]) - np.array([fx, fy])) < 20:
                    found = True
                    # 如果当前圆的得分更高，替换已有的圆
                    if score > fscore:
                        filtered[i] = (x, y, r, score)
                    break
            if not found:
                filtered.append((x, y, r, score))

        # 只保留得分最高的圆
        if filtered:
            best_circle = max(filtered, key=lambda x: x[3])
            return np.array([[best_circle[0], best_circle[1], best_circle[2]]])
        else:
            return None

    #-------------------------------------------------
    #--------------------任务驱动类--------------------
    #-------------------------------------------------

    # 过滤轮廓，并执行检测
    def detect_contours(self, contours, frame):
        frame_copy = frame.copy()
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour) # 外接矩形
            if 0.8 < w / h < 1.25:
                frame_roi = frame[y-5:y + h+5,x-5:x + w+5].copy()
                if frame_roi.size > 0:
                    self.hsv_detect(frame_copy, frame_roi, contour)
        return frame_copy

    # hsv空间的霍夫检测
    def hsv_detect(self, frame_copy, roi_img, contour):
        x, y, w, h = cv2.boundingRect(contour)

        hsv_colors = {
            #"blue": ([90, 50, 50], [130, 255, 255]),
            #"green": ([40, 50, 50], [80, 255, 255]),
            "red1":  ([0, 50, 50], [10, 255, 255]),  # 低端红色范围
            "red2":  ([170, 50, 50], [180, 255, 255]),  # 高端红色范围
            "yellow":([20, 100, 100], [40, 255, 255])
        }

        hsv_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in hsv_colors.items():
            hsv_mask = cv2.inRange(hsv_img, np.array(lower), np.array(upper))
            hsv_result = cv2.bitwise_and(roi_img, roi_img, hsv_mask)
            if cv2.countNonZero(hsv_mask) < 1000:
                continue
            hsv_open = cv2.morphologyEx(hsv_result, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            hsv_gray = cv2.cvtColor(hsv_open, cv2.COLOR_BGR2GRAY)
            hsv_edges = cv2.Canny(hsv_gray, 100, 200)
            _, hsv_thresh = cv2.threshold(hsv_edges, 150, 255, cv2.THRESH_BINARY)
            hsv_contours, _ = cv2.findContours(hsv_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            valid_contours = [cnt for cnt in hsv_contours if cv2.contourArea(cnt) > 1000]
            possible_contours = CVTools.filter_contours_by_centroid(valid_contours, min_dist=20)
            
            for contour in possible_contours:
                # 霍夫圆检测
                # param1用于边缘Canny算子的高阈值。大值检测更少的边缘，减少圆数量。
                # param2用于圆心的累加器阈值。小值更多的累加器投票，检测到更多的假阳性圆。
                circle = cv2.HoughCircles(hsv_edges, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
                                        param1=10, param2=33, minRadius=20, maxRadius=0)
                #circle = CVTools.filter_best_circle(circle)
                if circle is not None:
                    M = cv2.moments(contour)
                    center_x = int(M['m10'] / M['m00'])
                    center_y = int(M['m01'] / M['m00']) # 轮廓中心
                    circle = np.uint16(np.around(circle))
                    for i in circle[0, :]:
                        #cv2.circle(frame_copy, center=(x-5+i[0], y-5+i[1]), radius=i[2], color=(255, 0, 255), thickness=2)
                        cv2.putText(frame_copy, f"1", (x-5+i[0] - 40, y-5+i[1] - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                        frame_copy = CVTools.mark(contour, frame_copy, x, y)
                        if color_name == "red1" or color_name == "red2":
                            self.node.msg.is_circle_detected = True
                            self.node.msg.center_x2_error = int(y-5+i[1]) - frame_copy.shape[0]//2
                            self.node.msg.center_y2_error = int(x-5+i[0]) - frame_copy.shape[1]//2
                 
                # 矩形拟合
                approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True) # 逼近精度4%轮廓周长
                if len(approx) == 4:
                    M = cv2.moments(contour)
                    center_x = int(M['m10'] / M['m00'])
                    center_y = int(M['m01'] / M['m00']) # 轮廓中心
                    cv2.putText(frame_copy, f"2", (x+center_x-40, y+center_y-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    frame_copy = CVTools.mark(contour, frame_copy, x, y)
                    if color_name == "yellow":
                        self.node.msg.is_square_detected = True
                        self.node.msg.center_x1_error = int(y-5+center_y) - frame_copy.shape[0]//2
                        self.node.msg.center_y1_error = int(x-5+center_x) - frame_copy.shape[1]//2
                        
    
    # 霍夫直线，TODO：卡尔曼滤波
    def line_detect(self, frame):
        frame_copy = cv2.cvtColor(frame.copy(), cv2.COLOR_GRAY2BGR)
        edges = cv2.Canny(frame, 50, 200, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=100, maxLineGap=50)
        
        if lines is None:
            self.node.msg.is_line_detected = False
            self.node.msg.lateral_error = 0
            self.node.msg.angle_error = 0.0
            return frame_copy

        # 找到最长的直线
        best_line = max(lines, key=lambda line: np.sqrt((line[0][2] - line[0][0])**2 + (line[0][3] - line[0][1])**2))
        x1, y1, x2, y2 = best_line[0]
        cv2.line(frame_copy, (x1, y1), (x2, y2), (0, 0, 255), 3)

        # lateral_error > 0：线路在无人机右侧，需要向右移动
        # lateral_error < 0：线路在无人机左侧，需要向左移动
        lateral_error = (x1 + x2)//2 - frame.shape[1]//2 # 线中心-图形中心
        
        # 控制器优化
        if abs(lateral_error) < 30:
            gain = 0
        else:  # 横向误差较大
            gain = 1
        lateral_error = lateral_error * gain

        # angle_error > 0：线路右偏，需要顺时针增加yaw
        # angle_error < 0：线路左偏，需要逆时针减少yaw
        line_angle = np.arctan2(y2 - y1, x2 - x1)
        line_angle_body = line_angle - np.pi / 2 # 转换为机体坐标系
        line_angle_body = line_angle_body % (2 * np.pi) # 归一化[0, 2π]和yaw同步
        desired_angle = 0  # 期望角度为0（垂直方向）
        angle_error = line_angle - desired_angle
        if angle_error > np.pi: # 消除大于π的下半圆歧义，转换为负值表示左偏
            angle_error -= 2 * np.pi

        # 控制器优化
        angle_error = angle_error / (np.pi / 2) # 归一化到 [0, 1]
        if abs(angle_error) < 0.1:
            gain = 0  # 0增益
        else:  # 角度误差较大
            gain = 0.3
        angle_error = np.tanh(angle_error) * gain # 转换为线性映射，并增益

        # 填充ros消息
        self.node.msg.is_line_detected = True
        self.node.msg.lateral_error = int(lateral_error)
        self.node.msg.angle_error = angle_error
        return frame_copy
