from __future__ import print_function
import collections
import logging
import util
import cv2
import numpy as np
import evaluation


class SimpleTracker:
    """Instances of SimpleTracker tracker track targets within a video sequence.

    Given a sequence of images (such as a video) and a target location for the initial video of the sequence, instances
    of SimpleTracker will generate a bounding box for each image that represents the instances best guess for the
    target location at that point.

    Attributes:
        locations (OrderedDict<image, bbox>): Dictionary mapping images to target locations, ordered as they appear in
            sequence.
    """

    BBOX_COLOR = (255, 0, 0)

    def __init__(self, sequence, detector, bbox=None, preview_target=False):
        """Initialize Tracker with an image sequence, target location and a means of tracking / detection.

        Args:
            sequence (ImageSequence): Sequence to track over.
            detector (Detector): Detection algorithm to use for tracking.
            bbox (int, int, int, int): Bounding box in (cx, cy, w, h) format.
            preview_target (boolean): Whether or not to preview target location before tracking commences.
        """

        self._sequence = sequence
        self._detector = detector
        self._subscribers = []
        self.initial_location = bbox if bbox is not None else self._prompt_for_target()
        self.initial_image_id, self.current_image = next(self._sequence)
        self.current_id = self.initial_image_id
        self.current_location = None
        self.locations = collections.OrderedDict()

        if preview_target:
            self._preview_target_location()

    # TODO: Utility function for debugging. Either remove or remove call in init for production.
    def _preview_target_location(self):
        bbox_image = np.copy(self.current_image)
        left, top, right, bottom = evaluation.BboxFormats.convert_bbox_format(self.initial_location,
                                                                             evaluation.BboxFormats.CCWH,
                                                                             evaluation.BboxFormats.TLBR)
        # draw bounding box.
        cv2.line(bbox_image, (left, top), (right, top), self.BBOX_COLOR, 4)
        cv2.line(bbox_image, (right, top), (right, bottom), self.BBOX_COLOR, 4)
        cv2.line(bbox_image, (right, bottom), (left, bottom), self.BBOX_COLOR, 4)
        cv2.line(bbox_image, (left, bottom), (left, top), self.BBOX_COLOR, 4)

        cv2.imshow("preview", bbox_image)
        cv2.waitKey()

    def get_location_format(self):
        return self._detector.get_bbox_format()

    def _prompt_for_target(self):
        """Prompt user for target bounding box.

        Launches an image viewer on which the user draws a bounding box.

        Returns:
            (int, int, int, int): bbox in (x, y, w, h) format.
        """

        tl, br = util.get_rect(self.current_image)
        return evaluation.BboxFormats.convert_bbox_format(tl + br, evaluation.BboxFormats.TLBR,
                                                          evaluation.BboxFormats.CCWH)

    def track(self):
        """Generate bounding boxes for each image in sequence."""

        logging.info("Initializing detector with first image and target location.")
        try:
            self._detector.set_target(self.current_image, self.initial_location)
        except Exception as e:
            logging.warn("Detector threw following exception on call to set_target: {0}".format(e))
            self.locations = [None] * len(self._sequence)
            return
        logging.info("Detector initialized.")

        logging.info("Beginning tracking.")
        for image_id, image in self._sequence:
            location = self._detector.detect(image)
            self.locations[image_id] = location
            self.current_id = image_id
            self.current_image = image
            self.current_location = location

            for subscriber in self._subscribers:
                subscriber.update(self)
        logging.info("Tracking finished.")

    def add_subscriber(self, subscriber):
        """Add a subscriber to be updated during tracking process.

        A subscriber is an object with an update(tracker) method. Where the `tracker`
        argument is an object of this class. Subscribers are objects that must be updated
        during the tracking process. Aggregate statistics should be calculated afterward
        (to minimize cpu / memory load during inference).

        Args:
            subscriber (subscriber): The subscriber to be added.

        Returns:
            None
        """

        self._subscribers.append(subscriber)
