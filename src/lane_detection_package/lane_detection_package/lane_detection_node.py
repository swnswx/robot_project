import cv2
import numpy as np
import os
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node
from lane_detection_msgs.msg import LaneDetection

# 도로의 실제 거리(meter)를 이미지 상의 픽셀로 변환하는 비율
ym_per_pix = 30 / 720  # y축 (세로 방향)에서의 meter-to-pixel 비율
xm_per_pix = 3.7 / 720  # x축 (가로 방향)에서의 meter-to-pixel 비율

def readVideo():
    return cv2.VideoCapture("/home/sw/robot_project/src/include/drive.mp4")

def processImage(inpImage):
    lower_white = np.array([0, 160, 10])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(inpImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inpImage, inpImage, mask=mask)
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh, (3, 3), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def perspectiveWarp(inpImage):
    img_size = (inpImage.shape[1], inpImage.shape[0])
    src = np.float32([[590, 440], [690, 440], [200, 640], [1000, 640]])
    dst = np.float32([[200, 0], [1200, 0], [200, 710], [1200, 710]])
    matrix = cv2.getPerspectiveTransform(src, dst)
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)
    return birdseye, minv

def slide_window_search(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    midpoint = int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    window_height = int(binary_warped.shape[0] / nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    return ploty, left_fit, right_fit, left_fitx, right_fitx

def measure_lane_curvature(ploty, leftx, rightx):
    y_eval = np.max(ploty)
    left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 2)

    left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    if leftx[0] - leftx[-1] > 60:
        curve_direction = 'Right Curve'
    elif leftx[-1] - leftx[0] > 60:
        curve_direction = 'Left Curve'
    else:
        curve_direction = 'Straight'

    return (left_curverad + right_curverad) / 2.0, curve_direction

def draw_lane_lines(original_image, warped_image, Minv, left_fitx, right_fitx, ploty):
    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return pts_mean, result

def offCenter(meanPts, inpFrame):
    mpts = meanPts[-1][-1][-2].astype(int)
    pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    direction = "left" if deviation < 0 else "right"

    return deviation, direction

def addText(img, radius, direction, deviation, devDirection):
    font = cv2.FONT_HERSHEY_TRIPLEX

    if direction != 'Straight':
        text = f'Radius of Curvature: {radius:.0f}m'
        text1 = f'Curve Direction: {direction}'
    else:
        text = 'Radius of Curvature: N/A'
        text1 = f'Curve Direction: {direction}'

    cv2.putText(img, text, (50, 100), font, 0.8, (0, 100, 200), 2, cv2.LINE_AA)
    cv2.putText(img, text1, (50, 150), font, 0.8, (0, 100, 200), 2, cv2.LINE_AA)

    deviation_text = f'Off Center: {abs(deviation):.3f}m to the {devDirection}'
    cv2.putText(img, deviation_text, (50, 200), font, 0.8, (0, 100, 200), 2, cv2.LINE_AA)

    return img

# ROS2 퍼블리셔 클래스 추가
class LaneDetectionPublisher(Node):
    def __init__(self):
        super().__init__('lane_detection_publisher')
        self.publisher_ = self.create_publisher(LaneDetection, 'lane_detection_info', 10)

    def publish_lane_info(self, curveRad, curveDir, deviation, directionDev):
        msg = LaneDetection()
        msg.curve_radius = curveRad
        msg.curve_direction = curveDir
        msg.deviation = deviation
        msg.direction_dev = directionDev
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    lane_detection_publisher = LaneDetectionPublisher()

    image = readVideo()

    while rclpy.ok():
        ret, frame = image.read()
        if not ret:
            break

        try:
            birdView, minverse = perspectiveWarp(frame)
            canny = processImage(birdView)
            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(canny)
            curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)
            meanPts, result = draw_lane_lines(frame, canny, minverse, left_fitx, right_fitx, ploty)
            deviation, directionDev = offCenter(meanPts, frame)
            finalImg = addText(result, curveRad, curveDir, deviation, directionDev)

            lane_detection_publisher.publish_lane_info(curveRad, curveDir, deviation, directionDev)

        except Exception as e:
            print(f"Error occurred: {e}")
            curveRad = 0
            curveDir = "Straight"
            deviation = 0
            directionDev = "center"
            finalImg = addText(frame, curveRad, curveDir, deviation, directionDev)

        cv2.imshow("Final", finalImg)
        if cv2.waitKey(1) == 13:
            break

    image.release()
    cv2.destroyAllWindows()
    lane_detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()