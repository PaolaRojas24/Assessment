#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
1) Camera image -> BEV transformation
2) HLS masking + removal of circular objects
3) Collect lane pixels using histogram / sliding window
4) Publish Poly-fit coefficients as topics
   - /lane/left_fit  (Float32MultiArray, a1·b1)
   - /lane/right_fit (Float32MultiArray, a3·b3·c3·d3)
5) Publish BEV & debug images
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import warnings
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

# Fixed parameters ---------------------------------------------------------------
fixed_points = [(10, 450), (540, 430), (435, 240), (190, 240)]
x_m_per_pixel = 0.0016472868217054263
y_m_per_pixel = 0.003703703703703704

# Global memory variables (Hold histogram base points) ------------------------------
prev_left_base, prev_right_base = None, None
warnings.filterwarnings("ignore", category=np.RankWarning)

class LaneDetector(Node):
    def __init__(self):
        super().__init__("lane_detector")
        self.bridge = CvBridge()

        # Publishers
        self.pub_left  = self.create_publisher(Float32MultiArray, "/lane/left_fit", 1)
        self.pub_right = self.create_publisher(Float32MultiArray, "/lane/right_fit", 1)
        self.pub_bev   = self.create_publisher(Image, "/car/bev_image", 1)
        self.pub_dbg   = self.create_publisher(Image, "/car/sliding_window_image", 1)

        # Subscriber
        self.sub_image = self.create_subscription(Image, "/usb_cam/image_raw", self.cb_image, 1)

    # Utils ──────────────────────────────────────────────────────────────────────
    @staticmethod
    def get_bird_eye_view(img, out_size, pts):
        h, w = out_size[1], out_size[0]
        M = cv2.getPerspectiveTransform(np.float32([pts[0], pts[1], pts[3], pts[2]]),
                                        np.float32([[0, h], [w, h], [0, 0], [w, 0]]))
        return cv2.warpPerspective(img, M, out_size)

    @staticmethod
    def mask_hls(img, lower, upper, ksize=(21, 21)):
        hls = cv2.cvtColor(cv2.GaussianBlur(img, ksize, 0), cv2.COLOR_BGR2HLS)
        return cv2.inRange(hls, np.array(lower), np.array(upper))

    @staticmethod
    def remove_circles(mask, rmin=25, rmax=56):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if len(c) < 5: continue
            (_, _), r = cv2.minEnclosingCircle(c)
            if rmin < r < rmax:
                cv2.drawContours(mask, [c], -1, 0, -1)
        return mask

    # Sliding window ─────────────────────────────────────────────────────────────
    def sliding_window(self, bev, r_n=18, l_n=6, r_margin=60, l_margin=150,
                       minpix=50, disc_win=2):
        global prev_left_base, prev_right_base
        # HLS Masking
        mask = self.mask_hls(bev, [0, 210, 0], [180, 255, 90])
        mask = self.remove_circles(mask)

        # Base points using histogram
        hist = np.sum(mask[mask.shape[0]*4//5:, :], axis=0)
        mid = hist.shape[0]//2
        leftx_base  = np.argmax(hist[:mid])
        rightx_base = np.argmax(hist[mid:]) + mid

        l_bound = bev.shape[1]//3
        r_bound = int(1.7 * bev.shape[1]/3)
        use_left  = l_bound >= leftx_base  > 0
        use_right = rightx_base >= r_bound

        if not use_left and prev_left_base is not None: 
            leftx_base = prev_left_base
        else:                                            
            prev_left_base = leftx_base
            
        if not use_right and prev_right_base is not None: 
            rightx_base = prev_right_base
        else:                                            
            prev_right_base = rightx_base

        # Window heights
        l_h = mask.shape[0]//l_n
        r_h = mask.shape[0]//r_n
        lx_cur, rx_cur = leftx_base, rightx_base

        nz_y, nz_x = mask.nonzero()
        out = np.dstack((mask, mask, mask))

        lx, ly, rx, ry = [], [], [], []
        r_disc, last_r_idx, disc_found, r_pts = 0, None, False, 0

        for win in range(max(r_n, l_n)):
            # Left/Right window boundaries
            ly_low, ly_high = mask.shape[0]-(win+1)*l_h, mask.shape[0]-win*l_h
            ry_low, ry_high = mask.shape[0]-(win+1)*r_h, mask.shape[0]-win*r_h
            lx_low, lx_high = lx_cur-l_margin, lx_cur+l_margin
            rx_low, rx_high = rx_cur-r_margin, rx_cur+r_margin

            # Left ----------------------------------------------------------------
            if ly_high >= mask.shape[0]//4 and use_left:
                good = ((nz_y >= ly_low) & (nz_y < ly_high) &
                        (nz_x >= lx_low) & (nz_x < lx_high)).nonzero()[0]
                if len(good) > minpix:
                    lx_cur = int(np.mean(nz_x[good]))
                    lx.append(lx_cur); ly.append(int(np.mean(nz_y[good])))
                    cv2.rectangle(out, (lx_low, ly_low), (lx_high, ly_high), (255,0,0), 2)
                    cv2.circle(out, (lx_cur, (ly_low+ly_high)//2), 4, (0,255,0), -1)

            # Right ---------------------------------------------------------------
            if use_right:
                good = ((nz_y >= ry_low) & (nz_y < ry_high) &
                        (nz_x >= rx_low) & (nz_x < rx_high)).nonzero()[0]
                if len(good) > minpix:
                    if disc_found: continue
                    if last_r_idx is not None and win-last_r_idx > 1:
                        r_disc += win-last_r_idx-1
                    if r_disc > disc_win:
                        disc_found = True
                        continue
                    rx_cur = int(np.mean(nz_x[good]))
                    rx.append(rx_cur); ry.append(int(np.mean(nz_y[good])))
                    last_r_idx = win;  r_pts += len(good)
                    cv2.rectangle(out,(rx_low,ry_low),(rx_high,ry_high),(0,0,255),2)
                    cv2.circle(out,(rx_cur,(ry_low+ry_high)//2),4,(0,255,0),-1)

        if r_pts <= 5: rx, ry = [], []     # Ignore if insufficient right lane points
        return out, lx, ly, rx, ry

    # ────────────────────────────────────────────────────────────────────────────
    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        bev = self.get_bird_eye_view(frame, (frame.shape[1], frame.shape[0]), fixed_points)
        dbg, lx, ly, rx, ry = self.sliding_window(bev)

        left_fit_msg, right_fit_msg = Float32MultiArray(), Float32MultiArray()
        
        # Convert pixel -> meter and apply poly-fit
        if len(lx) >= 2:
            lfit = np.polyfit((450-np.array(ly))*y_m_per_pixel,
                              (275-np.array(lx))*x_m_per_pixel, 1)
            left_fit_msg.data = lfit.tolist()
            self.pub_left.publish(left_fit_msg)

        if len(rx) >= 4:
            rfit = np.polyfit((450-np.array(ry))*y_m_per_pixel,
                              (275-np.array(rx))*x_m_per_pixel, 3)
            right_fit_msg.data = rfit.tolist()
            self.pub_right.publish(right_fit_msg)

        # Debug images
        try:
            self.pub_bev.publish(self.bridge.cv2_to_imgmsg(bev, "bgr8"))
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()