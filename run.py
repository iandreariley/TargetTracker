import argparse
import logging
import detector
import tracker
import image_sequence
import evaluation
import os
import csv
import util
import timeit
import collections

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


def get_cli_args():
    """Parse command line arguments and return an argument namespace."""

    parser = argparse.ArgumentParser(description="Run a target-tracking algorithm over an image sequence.")

    parser.add_argument("sequence_source", help="The image sequence (e.g. device name, directory, ...) to run "
                                                "the tracking algorithm on.")
    parser.add_argument("--sequence_type", help="The type of image sequence (stream or directory).",
                        choices=[STREAM, DIRECTORY], default=DIRECTORY)
    parser.add_argument("--detection_algo", help="The algorithm used to detect the target ('nn' or 'cmt').",
                        choices=[NEURAL_NETWORK, FEATURE_BASED], default=NEURAL_NETWORK)
    parser.add_argument("--image_format", help="The file extension fort the image format used. Only applicable if the "
                                             "sequence drawn from image files. Ignored otherwise.", default=JPEG)
    parser.add_argument("--target_location", help="Bounding box giving location of target in first frame.", default="")
    parser.add_argument("--benchmark", action="store_true", help="Flag. If set, sequence_source is taken to be a "
                                                                 "directory with a number of sequences over which the "
                                                                 "tracking algorithm is meant to be run.")
    return parser.parse_args()


def configure_tracker(sequence_type, sequence_source, target_location, detection_algo):
    """Create correct tracker, detector and image-source object types based on CLI arguments

    Args:
        sequence_type (str): String indicating the type of sequence (e.g. video). One of {STREAM, DIRECTORY}.
        sequence_source (str): String for accessing video source (e.g. directory path)
        target_location (int, int, int, int): Bounding box for initial target location, or None.
        detection_algo (str): One of {FEATURE_BASED or NEURAL_NETWORK}. The method of object detection in each frame.

    Returns:
        tracker (tracker.SimpleTracker): The tracker object with the image source and detector type specified.
    """

    # construct image sequence
    sequence = None
    if sequence_type == DIRECTORY:
        sequence = image_sequence.DirectorySequence(sequence_source)
    else:
        logging.info("Images from streams are not currently supported")
        exit(0)

    # construct detector
    if detection_algo == NEURAL_NETWORK:
        detection_algo = detector.SiamFC()
    else:
        detection_algo = detector.CmtDetector()

    trkr = tracker.SimpleTracker(sequence, detection_algo, target_location)
    trkr.add_subscriber(evaluation.TrackingViewer())
    return trkr


def run_single_session(args):
    target_tracker = configure_tracker(args.sequence_type, args.sequence_source, args.target_location, args.detection_algo)
    target_tracker.track()
    return target_tracker


def load_groundtruth(directory):
    with open(os.path.join(directory, "groundtruth.txt")) as bbox_csv:
        reader = csv.reader(bbox_csv)
        ground_truth_bboxes = map(lambda region: util.region_to_bbox(map(float(region))), reader)
    return ground_truth_bboxes


def aggregate_results(results):
    pass


def run_benchmark(args):
    benchmark_sequences = os.listdir(args.sequence_source)
    benchmark_results = collections.OrderedDict()
    for sequence in benchmark_sequences:
        sequence_path = os.path.join(args.sequence_source, sequence)
        ground_truth = load_groundtruth(sequence_path)
        initial_location = ground_truth[0]
        target_tracker = configure_tracker(args.sequence_type, sequence_path, initial_location, args.detection_algo)

        start_time = timeit.default_timer()
        target_tracker.track()
        elapsed = timeit.default_timer - start_time

        results = evaluation.TrackingResults(target_tracker.locations, initial_location, elapsed, ground_truth[1:])
        results.add_metric(evaluation.TorrMetrics)
        results.add_metric(evaluation.FpsMetric)
        benchmark_results[sequence] = results
    return aggregate_results(benchmark_results)


def main():
    logging.basicConfig(level=logging.INFO)
    args = get_cli_args()

    if args.benchmark:
        run_benchmark(args)
    else:
        run_single_session(args)

