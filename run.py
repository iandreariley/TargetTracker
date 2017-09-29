import argparse
import logging
import detector
import tracker
import image_sequence
import ast
import util
import numpy as np

STREAM = 'stream'
DIRECTORY = 'directory'
NEURAL_NETWORK = 'nn'
FEATURE_BASED = 'cmt'
JPEG = "jpg"
SEQUENCE_TYPE = 'sequence_type'
SEQUENCE_SOURCE = 'sequence_source'
ALGO = 'detection_algo'
IMAGE_FORMAT = 'image_format'
TARGET_LOCATION = 'target_location'


def parse_args():
    """Parse command line arguments and return a dictionary"""

    parser = argparse.ArgumentParser(description="Run a target-tracking algorithm over an image sequence.")

    parser.add_argument("sequence_source", help="The image sequence (e.g. device name, directory, ...) to run "
                                                "the tracking algorithm on.")
    parser.add_argument("sequence_type", help="The type of image sequence (stream or directory).",
                        choices=[STREAM, DIRECTORY], default=DIRECTORY)
    parser.add_argument("detection_algo", help="The algorithm used to detect the target ('nn' or 'cmt').",
                        choices=[NEURAL_NETWORK, FEATURE_BASED], default=FEATURE_BASED)
    parser.add_argument("image_format", help="The file extension fort the image format used. Only applicable if the "
                                             "sequence drawn from image files. Ignored otherwise.", default=JPEG)
    parser.add_argument("target_location", help="Bounding box giving location of target in first frame.", default="")
    return parser.parse_args()


def configure_app(cli_args):
    """Create correct tracker, detector and image-source object types based on CLI arguments

    Args:
        cli_args (dict): Dictionary of command-line arguments.

    Returns:
        tracker (tracker.SimpleTracker): The tracker object with the image source and detector type specified.
    """

    # construct image sequence
    sequence = None
    if cli_args[SEQUENCE_TYPE] == DIRECTORY:
        sequence = image_sequence.DirectorySequence(cli_args[SEQUENCE_SOURCE])
    else:
        logging.info("Images from streams are not currently supported")
        exit(0)

    # construct detector
    if cli_args[ALGO] == NEURAL_NETWORK:
        detection_algo = detector.SiamFC()
    else:
        detection_algo = detector.CmtDetector()

    return tracker.SimpleTracker(sequence, detection_algo, bbox)


args = parse_args()
target_tracker = configure_app(args)
target_tracker.track()
