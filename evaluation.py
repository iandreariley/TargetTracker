import cv2
import util
import numpy as np


class TrackingViewer:
    """Tracker subscriber that provides a viewing GUI for the tracking process"""

    def __init__(self, title="view"):
        self._title = title

    def update(self, tracker):
        tl, br = util.to_tl_br(tracker.current_location)
        image = np.copy(tracker.current_image)
        tl = tuple(map(int, tl))
        br = tuple(map(int, br))

        tr = (br[0], tl[1])
        bl = (tl[0], br[1])
        cv2.line(image, tl, tr, (255, 0, 0), 4)
        cv2.line(image, tr, br, (255, 0, 0), 4)
        cv2.line(image, br, bl, (255, 0, 0), 4)
        cv2.line(image, bl, tl, (255, 0, 0), 4)

        cv2.imshow(self._title, image)
        k = cv2.waitKey(10)


class TrackingResults:
    """Results of a tracking session.

    Attributes:
        predictions (list<(int, int, int, int)>): A list of predicted bounding boxes.
        ground_truth (list<(int, int, int, int)>): A list of ground-truth bounding boxes.
            Must be the same length as predictions.
        first_bbox (int, int, int, int): Initial bounding box (ground-truth)
        elapsed_time (int): Time elapsed during tracking session.
    """

    def __init__(self, predictions, first_bbox=None, time=None, ground_truth=None):
        self.first_bbox = first_bbox
        self.predictions = predictions
        self.ground_truth = ground_truth
        self.elapsed_time = elapsed_time
