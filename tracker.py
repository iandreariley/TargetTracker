from __future__ import print_function
import collections
import logging
import util


class SimpleTracker:
    """Instances of SimpleTracker tracker track targets within a video sequence.

    Given a sequence of images (such as a video) and a target location for the initial video of the sequence, instances
    of SimpleTracker will generate a bounding box for each image that represents the instances best guess for the
    target location at that point.

    Attributes:
        locations (OrderedDict<image, bbox>): Dictionary mapping images to target locations, ordered as they appear in
            sequence.
    """

    def __init__(self, sequence, detector, bbox=None):
        """Initialize Tracker with an image sequence, target location and a means of tracking / detection.

        Args:
            sequence (ImageSequence):
            detector (Detector):
            bbox (int, int, int, int):
        """

        self._sequence = sequence
        self._detector = detector
        self._subscribers = []
        self.current_id, self.current_image = next(self._sequence)
        self.current_location = bbox or self._prompt_for_target()
        self.locations = collections.OrderedDict({self.current_id: (self.current_image, self.current_location)})

    def _prompt_for_target(self):
        """Prompt user for target bounding box.

        Launches an image viewer on which the user draws a bounding box.

        Returns:
            (int, int, int, int): bbox in (x, y, w, h) format.
        """

        tl, br = util.get_rect(self.current_image)
        return util.to_xywh(tl, br)

    def track(self):
        """Generate bounding boxes for each image in sequence."""

        logging.info("Initializing detector with first image and target location.")
        self._detector.set_target(self.current_image, self.current_location)
        logging.info("Detector initialized.")

        logging.info("Beginning tracking.")
        for image_id, image in self._sequence:
            location = self._detector.detect(image)
            self.locations[image_id] = (image, location)
            self.current_id = image_id
            self.current_image = image
            self.current_location = location

            for subscriber in self._subscribers:
                subscriber.update(self)
        logging.info("Tracking finished.")
