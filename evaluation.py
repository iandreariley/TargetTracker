import cv2
import util
import numpy as np


class TrackingViewer:
    """Tracker subscriber that provides a viewing GUI for the tracking process"""

    BBOX_COLOR = (255, 0, 0)

    def __init__(self, title="view"):
        self._title = title

    def update(self, tracker):
        """Update viewer to show most recent image and bounding box.

        Args:
            tracker (tracker.SimpleTracker): Object that is running the tracking session.

        Returns:
            None
        """
        tl, br = util.to_tl_br(tracker.current_location)
        tl = tuple(map(int, tl))
        br = tuple(map(int, br))

        bbox_image = self._draw_bounding_box(tracker.current_image, tl, br)
        cv2.imshow(self._title, bbox_image)
        cv2.waitKey(10)

    def _draw_bounding_box(self, original_image, top_left, bottom_right):
        # draw on copy of image, not original.
        image = np.copy(original_image)

        # get the top-right and bottom-left corners
        top_right = (bottom_right[0], top_left[1])
        bottom_left = (top_left[0], bottom_right[1])

        # draw bounding box.
        cv2.line(image, top_left, top_right, self.BBOX_COLOR, 4)
        cv2.line(image, top_right, bottom_right, self.BBOX_COLOR, 4)
        cv2.line(image, bottom_right, bottom_left, self.BBOX_COLOR, 4)
        cv2.line(image, bottom_left, top_left, self.BBOX_COLOR, 4)

        return image


class TrackingResults:
    """Results of a tracking session.

    Attributes:
        predictions (list<(int, int, int, int)>): A list of predicted bounding boxes.
        ground_truth (list<(int, int, int, int)>): A list of ground-truth bounding boxes.
            Must be the same length as predictions.
        first_bbox (int, int, int, int): Initial bounding box (ground-truth)
        elapsed_time (int): Time elapsed during tracking session.
    """

    def __init__(self, predictions, first_bbox=None, elapsed_time=None, ground_truth=None):
        self.first_bbox = first_bbox
        self.predictions = predictions
        self.ground_truth = ground_truth
        self.elapsed_time = elapsed_time
