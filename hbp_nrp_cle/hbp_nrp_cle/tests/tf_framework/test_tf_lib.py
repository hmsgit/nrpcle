import unittest
import cv2
import numpy as np
import random
from hbp_nrp_cle.tf_framework import tf_lib

__author__ = 'Alessandro Ambrosano'


class TestTFLib(unittest.TestCase):

    def test_ball_position_finder(self):
        for i in range(0, 100):
            tf_lib.cam.set_image_size(640, 480)
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            ball_radius = random.randrange(10, 30)
            ball_pos = (
                random.randrange(0 + ball_radius + 1, 640 - ball_radius - 1),
                random.randrange(0 + ball_radius + 1, 480 - ball_radius - 1)
            )
            # Note: color is given in BGR
            cv2.circle(img, ball_pos, ball_radius, (23, 230, 23), -1)
            ang_cmptd = tf_lib.ball_position_finder(
                tf_lib.bridge.cv2_to_imgmsg(img)
            )
            ang_ideal = tf_lib.cam.pixel2angle(ball_pos[0], ball_pos[1])[0]
            self.assertLessEqual(abs(ang_cmptd - ang_ideal), 0.2)

    def test_camera(self):
        w, h = 640, 480
        c = tf_lib.Camera()

        self.assertFalse(c.set_image_size(-w, h))
        self.assertFalse(c.set_image_size(w, -h))
        self.assertTrue(c.set_image_size(w, h))

        self.assertEqual(w, c.width)
        self.assertEqual(h, c.height)

        for i in range(0, 1000):
            rand_px = (
                random.randrange(0, w),
                random.randrange(0, h)
            )

            x, y = c.pixel2metric(rand_px[0], rand_px[1])
            xm, ym = c.pixel2norm(rand_px[0], rand_px[1])
            a, e = c.pixel2angle(rand_px[0], rand_px[1])

            if rand_px[0] <= w / 2:
                self.assertLessEqual(x, 0)
                self.assertLessEqual(xm, 0)
                self.assertGreaterEqual(xm, -1)
                self.assertGreaterEqual(a, 0)
            else:
                self.assertGreaterEqual(x, 0)
                self.assertGreaterEqual(xm, 0)
                self.assertLessEqual(ym, 1)
                self.assertLessEqual(a, 0)

            if rand_px[1] <= h / 2:
                self.assertLessEqual(y, 0)
                self.assertLessEqual(ym, 0)
                self.assertGreaterEqual(ym, -1)
                self.assertGreaterEqual(e, 0)
            else:
                self.assertGreaterEqual(y, 0)
                self.assertGreaterEqual(ym, 0)
                self.assertLessEqual(ym, 1)
                self.assertLessEqual(e, 0)

            m2p = c.metric2pixel(x, y)
            n2p = c.norm2pixel(xm, ym)
            a2p = c.angle2pixel(a, e)

            self.assertLessEqual(abs(m2p[0] - rand_px[0]), 0.1)
            self.assertLessEqual(abs(m2p[1] - rand_px[1]), 0.1)
            self.assertLessEqual(abs(n2p[0] - rand_px[0]), 0.1)
            self.assertLessEqual(abs(n2p[1] - rand_px[1]), 0.1)
            self.assertLessEqual(abs(a2p[0] - rand_px[0]), 0.1)
            self.assertLessEqual(abs(a2p[1] - rand_px[1]), 0.1)

    def test_angle_pixel(self):
        c = tf_lib.Camera()
        c.set_image_size(20, 20)
        self.__test_angle_pixel(c, 20, 20)
        self.__test_angle_pixel(c, 0, 0)
        self.__test_angle_pixel(c, 0, 20)
        self.__test_angle_pixel(c, 10, 5)

    def __test_angle_pixel(self, c, x, y):
        u, v = c.pixel2angle(x, y)
        _x, _y = c.angle2pixel(u, v)

        self.assertAlmostEqual(_x, x)
        self.assertAlmostEqual(_y, y)

    def test_norm_pixel(self):
        c = tf_lib.Camera()
        c.set_image_size(20, 20)
        self.__test_norm_pixel(c, 20, 20)
        self.__test_norm_pixel(c, 0, 0)
        self.__test_norm_pixel(c, 0, 20)
        self.__test_norm_pixel(c, 10, 5)

    def __test_norm_pixel(self, c, x, y):
        u, v = c.pixel2norm(x, y)
        _x, _y = c.norm2pixel(u, v)

        self.assertAlmostEqual(_x, x)
        self.assertAlmostEqual(_y, y)

    def test_norm_angle(self):
        c = tf_lib.Camera()
        c.set_image_size(20, 20)
        self.__test_norm_angle(c, 1, 1)
        self.__test_norm_angle(c, 0, 0)
        self.__test_norm_angle(c, -1, 1)
        self.__test_norm_angle(c, 0.5, -1)

    def __test_norm_angle(self, c, x, y):
        u, v = c.norm2angle(x, y)
        t1, t2 = c.angle2pixel(u, v)
        _x, _y = c.pixel2norm(t1, t2)

        self.assertAlmostEqual(_x, x)
        self.assertAlmostEqual(_y, y)

    def test_norm_metric(self):
        c = tf_lib.Camera()
        c.set_image_size(20, 20)
        self.__test_norm_metric(c, 1, 1)
        self.__test_norm_metric(c, 0, 0)
        self.__test_norm_metric(c, -1, 1)
        self.__test_norm_metric(c, 0.5, -1)

    def __test_norm_metric(self, c, x, y):
        u, v = c.norm2metric(x, y)
        t1, t2 = c.metric2pixel(u, v)
        _x, _y = c.pixel2norm(t1, t2)

        self.assertAlmostEqual(_x, x)
        self.assertAlmostEqual(_y, y)

    def test_exp(self):
        self.assertAlmostEqual(1, tf_lib.exp_wrapper(0))
        self.assertAlmostEqual(1, tf_lib.exp_wrapper(1) * tf_lib.exp_wrapper(-1))


if __name__ == '__main__':
    unittest.main()
