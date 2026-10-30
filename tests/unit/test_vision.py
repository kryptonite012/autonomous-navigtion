"""Vision tests on synthetic images: lighting, similar colours, multiple
objects, partial occlusion and false detections."""

import cv2
import numpy as np
import pytest

from robot.vision.calibration import suggest_range, validate_range
from robot.vision.detector import ColorDetector, VisionParams
from robot.vision.tracker import CentroidTracker

BGR = {"red": (30, 30, 200), "green": (50, 170, 40), "blue": (200, 80, 20),
       "yellow": (30, 210, 230), "orange": (20, 125, 245)}


@pytest.fixture
def detector(cfg):
    return ColorDetector(VisionParams.from_config(cfg["vision"]))


def scene(objs, bg=(120, 130, 140), brightness=1.0, noise=4, seed=0):
    img = np.full((480, 640, 3), bg, np.uint8)
    for color, (x, y), r in objs:
        cv2.circle(img, (x, y), r, BGR[color], -1)
    img = np.clip(img.astype(np.float32) * brightness, 0, 255)
    img += np.random.default_rng(seed).normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


@pytest.mark.parametrize("color", list(BGR))
def test_each_colour_detected(detector, color):
    dets, _ = detector.detect(scene([(color, (320, 240), 40)]))
    assert [d.color for d in dets] == [color]
    d = dets[0]
    assert abs(d.centroid[0] - 320) < 3 and abs(d.centroid[1] - 240) < 3
    assert d.confidence > 0.6


@pytest.mark.parametrize("brightness", [0.55, 0.8, 1.0, 1.15])
def test_lighting_variation(detector, brightness):
    objs = [("red", (150, 200), 35), ("green", (320, 200), 35), ("blue", (490, 200), 35)]
    dets, _ = detector.detect(scene(objs, brightness=brightness))
    assert sorted(d.color for d in dets) == ["blue", "green", "red"]


def test_similar_colours_red_vs_orange(detector):
    dets, _ = detector.detect(scene([("red", (200, 240), 40), ("orange", (440, 240), 40)]))
    by_x = sorted(dets, key=lambda d: d.centroid[0])
    assert [d.color for d in by_x] == ["red", "orange"]


def test_multiple_objects_same_colour(detector):
    objs = [("blue", (100 + 110 * i, 300), 30) for i in range(5)]
    dets, _ = detector.detect(scene(objs))
    assert len(dets) == 5 and all(d.color == "blue" for d in dets)


def test_partially_occluded_object_lower_confidence(detector):
    full, _ = detector.detect(scene([("green", (320, 240), 45)]))
    img = scene([("green", (320, 240), 45)])
    cv2.rectangle(img, (320, 180), (380, 300), (120, 130, 140), -1)   # occluder
    part, _ = detector.detect(img)
    assert len(part) == 1 and part[0].color == "green"
    edge, _ = detector.detect(scene([("green", (10, 240), 45)]))       # cut by image border
    assert edge and edge[0].confidence < full[0].confidence


def test_rejects_noise_and_tiny_blobs(detector):
    rng = np.random.default_rng(1)
    img = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    dets, _ = detector.detect(img)
    assert len(dets) <= 1
    tiny, _ = detector.detect(scene([("red", (320, 240), 4)]))
    assert tiny == []


def test_grey_background_no_detection(detector):
    dets, _ = detector.detect(scene([]))
    assert dets == []


def test_irregular_shape_filtered_by_solidity(detector):
    img = scene([])
    # comb shape: low solidity
    for x in range(200, 440, 40):
        cv2.rectangle(img, (x, 100), (x + 8, 320), BGR["yellow"], -1)
    cv2.rectangle(img, (200, 312), (440, 320), BGR["yellow"], -1)
    dets, _ = detector.detect(img)
    assert dets == []


def test_distance_only_when_calibrated(cfg):
    p = VisionParams.from_config(cfg["vision"])
    uncal = ColorDetector(p, 520, 0.11, -20, calibrated=False)
    cal = ColorDetector(p, 520, 0.11, -20, calibrated=True)
    img = scene([("red", (320, 400), 30)])
    assert uncal.detect(img)[0][0].distance_m is None
    assert cal.detect(img)[0][0].distance_m is not None


def test_tracker_keeps_ids_and_drops_blips(detector):
    tr = CentroidTracker(60, 3, min_hits=2)
    first = tr.update(detector.detect(scene([("red", (200, 200), 30)]))[0])
    assert first == []                                     # needs two sightings
    second = tr.update(detector.detect(scene([("red", (210, 205), 30)]))[0])
    assert len(second) == 1
    oid = second[0].object_id
    third = tr.update(detector.detect(scene([("red", (225, 210), 30)]))[0])
    assert third[0].object_id == oid
    for _ in range(5):
        tr.update([])
    assert tr.tracks == {}


def test_suggest_range_and_wraparound():
    img = scene([("blue", (320, 240), 60)], noise=2)
    r = suggest_range(img, (300, 220, 40, 40))
    assert len(r) == 1 and r[0].lower[0] <= 110 <= r[0].upper[0]
    red = np.zeros((40, 40, 3), np.uint8)
    hsv = np.zeros((40, 40, 3), np.uint8)
    hsv[:, :20] = (2, 220, 200)
    hsv[:, 20:] = (177, 220, 200)
    red = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    assert len(suggest_range(red, (0, 0, 40, 40))) == 2


def test_validate_range():
    validate_range([0, 0, 0], [179, 255, 255])
    with pytest.raises(ValueError):
        validate_range([10, 0, 0], [5, 255, 255])
    with pytest.raises(ValueError):
        validate_range([0, 0, 0], [200, 255, 255])


def test_tinted_floor_is_not_an_object(detector):
    # bluish floor with sensor noise: saturated enough to pass the blue HSV
    # range, but it is the background, not an object
    img = scene([], bg=(102, 75, 56), noise=12, seed=3)
    dets, _ = detector.detect(img)
    assert dets == []
    dets, _ = detector.detect(scene([("blue", (320, 240), 40)], bg=(102, 75, 56), seed=3))
    assert [d.color for d in dets] == ["blue"]
