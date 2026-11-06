import contextlib
import io
import sys
import unittest
from unittest import mock

import numpy as np

import ml_camera_harness as ml
from experimental_emergency_braking_following import eb_vision_node
from experimental_lane_assist import seg_lane_node


class CameraFramesTest(unittest.TestCase):
    def test_udp_latest_frame_drops_queued_packets(self):
        last = np.full((2, 3, 3), 255, dtype=np.uint8)
        sock = mock.Mock()
        sock.recv.side_effect = [b"old", b"new", BlockingIOError()]
        with mock.patch.object(ml.socket, "socket", return_value=sock), \
             mock.patch.object(ml.cv2, "imdecode", return_value=last) as decode:
            source = ml.frames_udp(5600, latest=True)
            self.assertIs(next(source)[1], last)
            source.close()
        self.assertEqual(decode.call_args.args[0].tobytes(), b"new")
        sock.close.assert_called_once()

    def test_reads_and_releases_camera(self):
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        capture = mock.Mock()
        capture.isOpened.return_value = True
        capture.read.return_value = (True, frame)
        with mock.patch.object(ml.cv2, "VideoCapture", return_value=capture) as open_camera:
            source = ml.frames_camera(0)
            name, result = next(source)
            self.assertEqual(name, "camera:0")
            self.assertIs(result, frame)
            source.close()
        open_camera.assert_called_once_with(0)
        capture.release.assert_called_once()

    def test_open_and_read_failures_are_reported_and_release(self):
        for opened in (False, True):
            with self.subTest(opened=opened):
                capture = mock.Mock()
                capture.isOpened.return_value = opened
                capture.read.return_value = (False, None)
                with mock.patch.object(ml.cv2, "VideoCapture", return_value=capture):
                    with self.assertRaisesRegex(RuntimeError, "camera 1"):
                        next(ml.frames_camera(1))
                capture.release.assert_called_once()


class SegmentationProfileTest(unittest.TestCase):
    def test_profile_preserves_masks_and_measures_all_stages(self):
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        output = np.array([[[0, 3], [4, 0]], [[2, 0], [1, 5]]], dtype=np.float32)
        extractor = mock.Mock()
        extractor.extract.return_value = (0, output)
        seg = ml.SegNcnn.__new__(ml.SegNcnn)
        seg.net = mock.Mock()
        seg.net.create_extractor.return_value = extractor
        extractor.input.return_value = 0
        seg.w, seg.h = 3, 2
        seg.letterbox = False
        seg.in_name, seg.out_da, seg.out_ll = "in0", "out0", "out1"
        with mock.patch.object(ml, "ncnn_mat", return_value=mock.sentinel.input):
            expected = seg.infer(frame)
            timings = {}
            actual = seg.infer(frame, timings)
        for before, after in zip(expected, actual):
            np.testing.assert_array_equal(before, after)
        self.assertEqual(set(timings), {"seg_preprocess", "seg_ncnn_extract", "seg_postprocess"})
        self.assertTrue(all(value >= 0 for value in timings.values()))
        self.assertEqual(extractor.extract.call_count, 4)

    def test_extract_failure_is_reported(self):
        seg = ml.SegNcnn.__new__(ml.SegNcnn)
        seg.net = mock.Mock()
        seg.net.create_extractor.return_value.input.return_value = 0
        seg.net.create_extractor.return_value.extract.return_value = (-1, None)
        seg.w, seg.h = 3, 2
        seg.letterbox = False
        seg.in_name, seg.out_da, seg.out_ll = "in0", "out0", "out1"
        with mock.patch.object(ml, "ncnn_mat"):
            with self.assertRaisesRegex(RuntimeError, "out0"):
                seg.infer(np.zeros((2, 3, 3), dtype=np.uint8), {})

    def test_letterbox_removes_padding_from_masks(self):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)
        output = np.zeros((2, 384, 640), dtype=np.float32)
        output[1, 12:372, 320:] = 1
        extractor = mock.Mock()
        extractor.input.return_value = 0
        extractor.extract.return_value = (0, output)
        seg = ml.SegNcnn.__new__(ml.SegNcnn)
        seg.net = mock.Mock()
        seg.net.create_extractor.return_value = extractor
        seg.w, seg.h, seg.letterbox = 640, 384, True
        seg.in_name, seg.out_da, seg.out_ll = "in0", "out0", "out1"
        with mock.patch.object(ml, "ncnn_mat") as convert:
            da, ll = seg.infer(frame)
        padded = convert.call_args.args[0]
        self.assertEqual(padded.shape, (384, 640, 3))
        self.assertTrue((padded[:12] == 114).all())
        self.assertTrue((da[:, :320] == 0).all())
        self.assertTrue((da[:, 320:] == 1).all())
        np.testing.assert_array_equal(da, ll)

    def test_reports_measured_frames_after_warmup(self):
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        seg = mock.Mock()

        def infer(image, timings):
            timings.update(seg_preprocess=0.001, seg_ncnn_extract=0.002, seg_postprocess=0.003)
            return (np.zeros(image.shape[:2], dtype=np.uint8),) * 2

        seg.infer.side_effect = infer

        def frames(_):
            yield "camera", frame
            yield "camera", frame

        args = ["harness", "--backend", "ncnn", "--camera", "0", "--no-yolo", "--no-display",
                "--seg-param", "a.param", "--seg-bin", "a.bin", "--profile-seg",
                "--profile-warmup", "1", "--max-frames", "2"]
        output = io.StringIO()
        with contextlib.redirect_stdout(output), mock.patch.object(sys, "argv", args), \
             mock.patch.object(ml, "frames_camera", side_effect=frames), \
             mock.patch.object(ml, "SegNcnn", return_value=seg):
            ml.main()
        text = output.getvalue()
        self.assertIn("Profile: last 1 frames", text)
        for name in ("input_wait", "seg_preprocess", "seg_ncnn_extract", "seg_postprocess",
                     "render_output", "frame_total"):
            self.assertIn(name, text)


class YoloProfileTest(unittest.TestCase):
    def test_profile_preserves_detections_and_checks_output_status(self):
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        extractor = mock.Mock()
        extractor.input.return_value = 0
        extractor.extract.return_value = (0, np.zeros((16, 2, 2), dtype=np.float32))
        yolo = ml.YoloNcnn.__new__(ml.YoloNcnn)
        yolo.net = mock.Mock()
        yolo.net.create_extractor.return_value = extractor
        yolo.cfg = {"width": 8, "height": 8, "classes": 1, "anchor_num": 3}
        yolo.in_name, yolo.out_names, yolo.conf = "in0", ["out0", "out1"], 0.3
        with mock.patch.object(ml, "ncnn_mat"), \
             mock.patch.object(ml, "decode_yolo", return_value=[(0.9, 0, 1, 2, 3, 4)]):
            expected = yolo.infer(frame)
            timings = {}
            self.assertEqual(yolo.infer(frame, timings), expected)
            self.assertEqual(set(timings), {"yolo_preprocess", "yolo_ncnn_extract", "yolo_postprocess"})
            self.assertTrue(all(value >= 0 for value in timings.values()))
            extractor.extract.return_value = (-1, None)
            with self.assertRaisesRegex(RuntimeError, "out0"):
                yolo.infer(frame, {})


class InputRoutingTest(unittest.TestCase):
    def run_node(self, module, args, camera):
        frame = np.zeros((360, 640, 3), dtype=np.uint8)

        def frames(_):
            yield "test", frame

        seg = mock.Mock()
        seg.infer.return_value = (np.zeros((360, 640), dtype=np.uint8),) * 2
        yolo = mock.Mock()
        yolo.infer.return_value = []
        with contextlib.redirect_stdout(io.StringIO()), \
             mock.patch.object(sys, "argv", [module.__file__, *args]), \
             mock.patch.object(ml, "frames_camera", side_effect=frames) as camera_source, \
             mock.patch.object(ml, "frames_udp", side_effect=frames) as udp_source, \
             mock.patch.object(ml, "SegNcnn", return_value=seg), \
             mock.patch.object(ml, "YoloNcnn", return_value=yolo):
            module.main()
        if camera:
            camera_source.assert_called_once_with(0)
            udp_source.assert_not_called()
        else:
            camera_source.assert_not_called()
            udp_source.assert_called_once()

    def test_camera_and_udp_selection_for_each_entrypoint(self):
        cases = (
            (ml, ["--backend", "ncnn", "--no-display", "--max-frames", "1"]),
            (seg_lane_node, ["--backend", "ncnn", "--no-display", "--max-frames", "1",
                             "--odom-port", "0", "--state-udp", ""]),
            (eb_vision_node, ["--backend", "ncnn"]),
        )
        for module, args in cases:
            for camera in (False, True):
                with self.subTest(module=module.__name__, camera=camera):
                    self.run_node(module, args + (["--camera", "0"] if camera else []), camera)


if __name__ == "__main__":
    unittest.main()
