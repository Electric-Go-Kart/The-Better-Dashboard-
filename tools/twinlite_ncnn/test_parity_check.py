import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import parity_check


class ValidationMetricsTest(unittest.TestCase):
    def test_mask_unpadding_and_baseline_equivalence(self):
        rng = np.random.default_rng(42)
        baseline = rng.standard_normal((2, 384, 640)).astype(np.float32)
        actual, _ = parity_check.masks(baseline, baseline)
        np.testing.assert_array_equal(actual, baseline.argmax(0)[12:-12])

        logits = np.zeros((2, 192, 320), dtype=np.float32)
        logits[1, 6:186, 160:] = 1
        actual, _ = parity_check.masks(logits, logits)
        self.assertEqual(actual.shape, (360, 640))
        self.assertTrue((actual[:, :320] == 0).all())
        self.assertTrue((actual[:, 320:] == 1).all())

    def test_metrics_and_empty_class(self):
        metric = parity_check.Metric()
        truth = np.array([[0, 0], [1, 1]])
        pred = np.array([[0, 1], [1, 0]])
        metric.add(pred, truth, pred, truth)
        da, ll_acc, ll_iou = metric.summary()
        self.assertAlmostEqual(da, 1 / 3)
        self.assertAlmostEqual(ll_acc, 0.5)
        self.assertAlmostEqual(ll_iou, 1 / 3)

        metric = parity_check.Metric()
        zero = np.zeros((2, 2), dtype=np.int64)
        with np.errstate(divide="raise", invalid="raise"):
            metric.add(zero, zero, zero, zero)
        np.testing.assert_allclose(metric.summary(), (0.5, 0.5, 0.0))

    def test_load_labels_and_report_missing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            da_dir, ll_dir = root / "da", root / "ll"
            da_dir.mkdir()
            ll_dir.mkdir()
            da = np.full((360, 640), 127, dtype=np.uint8)
            ll = np.zeros((360, 640), dtype=np.uint8)
            self.assertTrue(cv2.imwrite(str(da_dir / "sample.png"), da))
            self.assertTrue(cv2.imwrite(str(ll_dir / "sample.png"), ll))
            actual_da, actual_ll = parity_check.load_gt("sample.jpg", str(da_dir), str(ll_dir))
            self.assertTrue((actual_da == 1).all())
            self.assertTrue((actual_ll == 0).all())
            with self.assertRaisesRegex(FileNotFoundError, "missing.png"):
                parity_check.load_gt("missing.jpg", str(da_dir), str(ll_dir))


if __name__ == "__main__":
    unittest.main()
