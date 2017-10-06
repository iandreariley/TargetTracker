import cv2
import util
import numpy as np
import logging
import pickle


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
        """Stores arguments as results.

        Args:
            predictions (list<(int, int, int, int)>): List of predicted bounding boxes.
            first_bbox (int, int, int, int): The given location of the target in the first frame of the sequence.
            elapsed_time (int): Clock time elapsed during tracking.
            ground_truth (list<(4 * int) | (8 * int)>): A list of ground-truth bounding boxes. If each element is
                (int, int, int, int) it is assumed to be a bounding box in (x, y, w, h) format, where (x, y) is the
                top left corner of the box. If each element is an 8-tuple of ints, it is assumed to be of the form
                (x1, y1, x2, y2 ... x4, y4) where each (xn, yn) is a corner of a quadrangle defining the region.
        """

        self.metrics = []
        self.predictions = predictions
        self.first_bbox = first_bbox
        self.elapsed_time = elapsed_time

        if ground_truth is not None and len(ground_truth[0]) == 8:
            self.ground_truth = map(util.region_to_bbox, ground_truth)
        else:
            self.ground_truth = ground_truth

    def add_metric(self, metric):
        """Add a metric to compute over these results"""
        self.metrics.append(metric)

    def get_metrics(self):
        """Returns the results of all metrics added to this tracking run."""

        metrics_results = {}
        for metric in self.metrics:
            for result in metric.get_metrics(self):
                name, value = result
                if name in metrics_results:
                    logging.warning('Multiple metrics with the name {0} were computed. Last computation overwrites all '
                                    'others.'.format(name))
                metrics_results[name] = value

        return metrics_results

    def save(self, path):
        """Serialize this object to file.

        Args:
            path (str): file path to save to.

        Returns:
            None

        Raises:
            FileNotFoundError if the path does not reference a valid file.
        """

        with open(path, 'rb') as pickle_file:
            pickle.dump(self, pickle_file)


class FpsMetric:
    """Computes fps"""

    METRICS = ['fps']

    def __init__(self):
        self._fps = None

    def get_metrics(self, results):
        """Implementation of metrics interface. Computes frames per second for results.

        Args:
            results (evaluation.TrackingResults): The tracking predictions over which to compute these metrics.
        """

        if self._fps is None:
            self._compute_fps(results)
        return zip(self.METRICS, [self._fps])

    def _compute_fps(self, results):
        n_frames = len(results.predictions)
        self._fps = n_frames / results.elapsed_time


class TorrMetrics:
    """Computes metrics of tracking performance used in SiamFC paper (Torr Vision at Oxford)."""

    METRICS = ["session_length", "precision", "precision_auc", "iou"]

    def __init__(self, distance_threshold):
        """Gets necessary thresholds for computation of metrics.

        Args:
            distance_threshold (float): The distance (in pixels?) that the center of the predicted bounding box may
                be from the center of the ground_truth bounding box and still be considered accurate.
        """

        self._distance_threshold = distance_threshold
        self._metrics = None

    def get_metrics(self, results):
        """Implementation of Metrics interface. returns key-value pairs of metrics

        Args:
            results (evaluation.TrackingResults): The tracking predictions over which to comput these metrics.
        """

        if self._metrics is None:
            self._compute_metrics(results)
        return self._metrics

    def _compute_metrics(self, results):
        predictions = np.array(results.predictions.values())
        ground_truth = np.array(results.ground_truth)
        length = len(predictions)
        new_distances = np.zeros(length)
        new_ious = np.zeros(length)
        n_thresholds = 50
        precisions_ths = np.zeros(n_thresholds)

        # Compute IoUs for each frame.
        for i in range(length):
            new_distances[i] = self._compute_distance(predictions[i, :], ground_truth[i, :])
            new_ious[i] = self._compute_iou(predictions[i, :], ground_truth[i, :])

        # what's the percentage of frame in which center displacement is inferior to given threshold? (OTB metric)
        precision = sum(new_distances < self._distance_threshold)/np.size(new_distances) * 100

        # find above result for many thresholds, then report the AUC
        thresholds = np.linspace(0, 25, n_thresholds+1)
        thresholds = thresholds[-n_thresholds:]
        # reverse it so that higher values of precision goes at the beginning
        thresholds = thresholds[::-1]
        for i in range(n_thresholds):
            precisions_ths[i] = sum(new_distances < thresholds[i])/np.size(new_distances)

        # integrate over the thresholds
        precision_auc = np.trapz(precisions_ths)

        # per frame averaged intersection over union (OTB metric)
        iou = np.mean(new_ious) * 100

        self._metrics = zip(self.METRICS, [length, precision, precision_auc, iou])

    def _compute_distance(self, boxA, boxB):
        a = np.array((boxA[0]+boxA[2]/2, boxA[1]+boxA[3]/2))
        b = np.array((boxB[0]+boxB[2]/2, boxB[1]+boxB[3]/2))
        dist = np.linalg.norm(a - b)

        assert dist >= 0
        assert dist != float('Inf')

        return dist


    def _compute_iou(self, boxA, boxB):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
        yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])

        if xA < xB and yA < yB:
            # compute the area of intersection rectangle
            interArea = (xB - xA) * (yB - yA)
            # compute the area of both the prediction and ground-truth
            # rectangles
            boxAArea = boxA[2] * boxA[3]
            boxBArea = boxB[2] * boxB[3]
            # compute the intersection over union by taking the intersection
            # area and dividing it by the sum of prediction + ground-truth
            # areas - the intersection area
            iou = interArea / float(boxAArea + boxBArea - interArea)
        else:
            iou = 0

        assert iou >= 0
        assert iou <= 1.01

        return iou
