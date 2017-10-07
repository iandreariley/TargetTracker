import argparse
import logging
import detector
import tracker
import image_sequence
import evaluation
import os
import csv
import util
import json
import time
import collections
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
    parser.add_argument("--evaluate", action="store_true", help="Flag. Whether to run against ground truth.")
    parser.add_argument("--benchmark", action="store_true", help="Flag. If set, sequence_source is taken to be a "
                                                                 "directory with a number of sequences over which the "
                                                                 "tracking algorithm is meant to be run.")
    parser.add_argument("--save_dir", help="directory in which to save output", default="")
    parser.add_argument("--save_name", help="filename for results output file.", default=".")
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


def shift_location_up_right(location):
        location[0] -= location[2] / 2
        location[1] -= location[3] / 2
        return location


def run_single_session(args):
    if args.evaluate:
        ground_truth = load_groundtruth(args.sequence_source)
        initial_location = ground_truth[0]
        logging.info("Initial location: {0}".format(initial_location))
        target_tracker = configure_tracker(args.sequence_type, args.sequence_source, initial_location,
                                           args.detection_algo)
        start_time = time.time()
        target_tracker.track()
        elapsed = time.time() - start_time

        results = evaluation.TrackingResults(target_tracker.locations, initial_location, elapsed, ground_truth,
                                             target_tracker.get_location_format())
        distance_threshold = load_params(os.path.join('siamfc-params', 'evaluation.json'))['dist_threshold']
        results.add_metric(evaluation.TorrMetrics(distance_threshold))
        results.add_metric(evaluation.FpsMetric())
        save_results(args, results)
        return aggregate_results([results]), distance_threshold, 1
    else:
        target_tracker = configure_tracker(args.sequence_type, args.sequence_source, args.target_location,
                                           args.detection_algo)
        target_tracker.track()
        return target_tracker


def load_groundtruth(directory):
    with open(os.path.join(directory, "groundtruth.txt")) as bbox_csv:
        reader = csv.reader(bbox_csv)
        ground_truth_bboxes = map(lambda region: map(int, util.region_to_bbox(np.array(map(float, region)))), reader)
    return ground_truth_bboxes


def get_dictionary_entry(dicts, entry_key):
    """For a list of dictionaries, return the value paired with a given key for each dictionary

    Args:
        dicts (iterable<dicts>): An iterable of dictionaries. All dictionaries must contain an entry for `entry_key`.
        entry_key (hashable): The key to search in all dictionaries in `dicts`.

    Returns:
        numpy.ndarray of the values matching `entry_key` for each dictionary in `dicts`.

    Raises:
        KeyError if any dictionary in `dicts` does not contain `entry_key`.
    """
    return np.array(map(lambda d: d[entry_key], dicts))


def weighted_average_metric(dicts, metric, weights):
    """Weighted average of metric in a set of metric dictionaries."""
    return np.average(get_dictionary_entry(dicts, metric), weights=weights)


def aggregate_results(results):
    metrics_dicts = map(lambda result: result.get_metrics(), results)
    lengths = get_dictionary_entry(metrics_dicts, "session_length")

    return weighted_average_metric(metrics_dicts, "precision", lengths), \
           weighted_average_metric(metrics_dicts, "precision_auc", lengths), \
           weighted_average_metric(metrics_dicts, "iou", lengths), \
           weighted_average_metric(metrics_dicts, "fps", lengths), \
           np.sum(lengths)


def load_params(path):
    with open(path) as json_file:
        params = json.load(json_file)
    return params


def run_benchmark(args):
    """Run and evaluate algorithm on benchmark dataset.

    Args:
        args (namespace): CLI arguments to program

    Returns:
        five-tuple of aggregate metrics, distance_threshold, and number of videos in benchmark.
    """
    benchmark_sequences = os.listdir(args.sequence_source)
    benchmark_results = collections.OrderedDict()
    distance_threshold = load_params(os.path.join('parameters', 'evaluation.json'))['dist_threshold']
    for sequence in benchmark_sequences:
        sequence_path = os.path.join(args.sequence_source, sequence)
        ground_truth = load_groundtruth(sequence_path)
        initial_location = ground_truth[0]
        target_tracker = configure_tracker(args.sequence_type, sequence_path, initial_location, args.detection_algo)

        start_time = time.time()
        target_tracker.track()
        elapsed = time.time() - start_time

        results = evaluation.TrackingResults(target_tracker.locations, initial_location, elapsed, ground_truth[1:])
        results.add_metric(evaluation.TorrMetrics(distance_threshold))
        results.add_metric(evaluation.FpsMetric())
        benchmark_results[sequence] = results
    return aggregate_results(benchmark_results), distance_threshold, len(benchmark_sequences)


def print_metrics(metrics, dist_threshold, nv):
    precision, auc, iou, fps, total_frames = metrics
    print '-- Overall stats (averaged per frame) on ' + str(nv) + ' videos (' + str(total_frames) + ' frames) --'
    print ' -- Precision ' + "(%d px)" % dist_threshold + ': ' + "%.2f" % precision + \
          ' -- Precisions AUC: ' + "%.2f" % auc + \
          ' -- IOU: ' + "%.2f" % iou + \
          ' -- Speed: ' + "%.2f" % fps + ' --'


def save_results(args, results):
    if args.save_name:
        results.save(os.path.join(args.save_dir, args.save_name))


def main():
    logging.basicConfig(level=logging.INFO)
    args = get_cli_args()

    if args.benchmark:
        logging.info("Running benchmark in directory {0} with {1} videos".format(args.sequence_source, len(os.listdir(args.sequence_source))))
        benchmark_results = run_benchmark(args)
        print_metrics(*benchmark_results)
    else:
        logging.info("Tracking object on source {0}".format(args.sequence_source))
        run_results = run_single_session(args)
        print_metrics(*run_results)


if __name__ == '__main__':
    main()
