import argparse
import logging
import detector
import tracker
import image_sequence
import evaluation
import os
import sys
import util
import json
import time
import collections
import numpy as np
from PIL import Image
from region_to_bbox import region_to_bbox


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

    # parser.add_argument("sequence_source", help="The image sequence (e.g. device name, directory, ...) to run "
    #                                             "the tracking algorithm on.")
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
    parser.add_argument("--save_dir", help="directory in which to save output. If  running a benchmark, "
            "This parameter must be supplied.", default="")
    parser.add_argument("--save_name", help="filename for results output file.", default=".")
    parser.add_argument("--db", action="store_true", help="Set for debugging output.")
    parser.add_argument("--visualize", action="store_true", help="Visualize tracking.")
    parser.add_argument("--preview", action="store_true", help="Preview target before beginning tracking")
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
    detection_algo = get_detector(detection_algo)

    trkr = tracker.SimpleTracker(sequence, detection_algo, target_location)
    trkr.add_subscriber(evaluation.TrackingViewer())
    return trkr


def get_detector(detection_algo_type, hp, design, env):
    if detection_algo_type == NEURAL_NETWORK:
        return detector.SiameseNetwork(hp, design, env)
    else:
        return detector.CmtDetector()


def evaluate_detector_on_sequence(detection_algo, sequence_path, visualize, preview=False):
    """Evaluates a detection method (a tracking algorithm) over a single sequence.

    Sequence is expected to be a directory in the file system. The images that make up the sequence should be stored as
    jpg files with integer names that specify their ordering. The directory `sequence_path` should also contain a file
    named `groundtruth.txt` that contains the ground truth bounding box for each frame. It should be in csv format.

    Args:
        detection_algo (detector.Detector): Detector object to evaluate.
        sequence_path (str): Path to sequence.
        visualize (boolean): Whether or not to add a viewer to the tracker during tracking.
        preview (boolean): Whether or not to preview target before tracking commences.

    Returns:
        TrackingResults: The results object generated by evaluating the `detector` over the sequence.
        int: The distance threshold used to measure precision
    """
    ground_truth = load_groundtruth(sequence_path)
    initial_location = ground_truth[0]
    logging.debug("Initial location: {0}".format(initial_location))
    sequence = image_sequence.DirectorySequence(sequence_path)
    target_tracker = tracker.SimpleTracker(sequence, detection_algo, initial_location, preview)

    if visualize:
        target_tracker.add_subscriber(evaluation.TrackingViewer())

    start_time = time.time()
    target_tracker.track()
    elapsed = time.time() - start_time

    results = evaluation.TrackingResults(target_tracker.locations, initial_location, elapsed, ground_truth,
                                         target_tracker.get_location_format())
    distance_threshold = load_params(os.path.join('siamfc-params', 'evaluation.json'))['dist_threshold']
    results.add_metric(evaluation.TorrMetrics(distance_threshold))
    results.add_metric(evaluation.FpsMetric())
    return results, distance_threshold


def run_single_session(args):
    if args.evaluate:
        detection_algo = get_detector(args.detection_algo)
        results, distance_threshold = evaluate_detector_on_sequence(detection_algo, args.sequence_source, args.visualize, args.preview)
        save_results(args, results)
        return aggregate_results([results]), distance_threshold, 1
    else:
        target_tracker = configure_tracker(args.sequence_type, args.sequence_source, args.target_location,
                                           args.detection_algo)
        target_tracker.track()
        return target_tracker


def alt_run():
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
    hp, ev, run, env, design = parse_arguments()
    validation_path = os.path.join(env.root_dataset, ev.dataset)
    videos = filter(lambda v: os.path.isdir(os.path.join(validation_path, v)), os.listdir(validation_path))
    d = detector.CmtDetector()
    for video in videos:
        # run tracking
        results_filename = video + '_results.p'
        results_path = os.path.join(ev.save_dir, results_filename)

        if os.path.exists(results_path):
            continue
        try:
            gt, frame_name_list, _, _ = _init_video(env, ev, video)
            bbox = region_to_bbox(gt[ev.start_frame], center=False)
            t = tracker.Tracker(hp, run, design, frame_name_list, bbox, d)
            bboxes, speed = t.track()
            _, precision, precision_auc, iou, _ = _compile_results(gt, bboxes, ev.dist_threshold)

            # save results
            results = evaluation.SimpleResults(bboxes, speed, map(lambda bb: region_to_bbox(bb, center=False), gt))
            results.save(results_path)
        except Exception as e:
            logging.warn("Sequence {0} threw the following exception: {1}".format(video, e))


def _compile_results(gt, bboxes, dist_threshold):
    l = len(bboxes.values())
    gt4 = np.zeros((l, 4))
    new_distances = np.zeros(l)
    new_ious = np.zeros(l)
    n_thresholds = 50
    precisions_ths = np.zeros(n_thresholds)

    for i in range(l):
        gt4[i, :] = region_to_bbox(gt[i, :], center=False)
        new_distances[i] = _compute_distance(np.array(bboxes.values()[i]), gt4[i, :])
        new_ious[i] = _compute_iou(np.array(bboxes.values()[i]), gt4[i, :])

    # what's the percentage of frame in which center displacement is inferior to given threshold? (OTB metric)
    precision = sum(new_distances < dist_threshold)/np.size(new_distances) * 100

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

    return l, precision, precision_auc, iou, gt4


def _init_video(env, evaluation, video):
    video_folder = os.path.join(env.root_dataset, evaluation.dataset, video)
    frame_name_list = [f for f in os.listdir(video_folder) if f.endswith(".jpg")]
    frame_name_list = [os.path.join(env.root_dataset, evaluation.dataset, video, '') + s for s in frame_name_list]
    frame_name_list.sort()
    with Image.open(frame_name_list[0]) as img:
        frame_sz = np.asarray(img.size)
        frame_sz[1], frame_sz[0] = frame_sz[0], frame_sz[1]

    # read the initialization from ground truth
    gt_file = os.path.join(video_folder, 'groundtruth.txt')
    gt = np.genfromtxt(gt_file, delimiter=',')
    n_frames = len(frame_name_list)
    assert n_frames == len(gt), 'Number of frames and number of GT lines should be equal.'

    return gt, frame_name_list, frame_sz, n_frames


def _compute_distance(boxA, boxB):
    a = np.array((boxA[0]+boxA[2]/2, boxA[1]+boxA[3]/2))
    b = np.array((boxB[0]+boxB[2]/2, boxB[1]+boxB[3]/2))
    dist = np.linalg.norm(a - b)

    assert dist >= 0
    assert dist != float('Inf')

    return dist


def _compute_iou(boxA, boxB):
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


def parse_arguments(in_hp={}, in_evaluation={}, in_run={}):
    with open('../siamfc-tf/parameters/hyperparams.json') as json_file:
        hp = json.load(json_file)
    with open('../siamfc-tf/parameters/evaluation.json') as json_file:
        ev = json.load(json_file)
    with open('../siamfc-tf/parameters/run.json') as json_file:
        run = json.load(json_file)
    with open('../siamfc-tf/parameters/environment.json') as json_file:
        env = json.load(json_file)
    with open('../siamfc-tf/parameters/design.json') as json_file:
        design = json.load(json_file)

    for name, value in in_hp.iteritems():
        hp[name] = value
    for name, value in in_evaluation.iteritems():
        ev[name] = value
    for name, value in in_run.iteritems():
        run[name] = value

    hp = collections.namedtuple('hp', hp.keys())(**hp)
    ev = collections.namedtuple('evaluation', ev.keys())(**ev)
    run = collections.namedtuple('run', run.keys())(**run)
    env = collections.namedtuple('env', env.keys())(**env)
    design = collections.namedtuple('design', design.keys())(**design)

    return hp, ev, run, env, design


def load_groundtruth(directory):
    return map(util.region_to_bbox, np.genfromtxt(os.path.join(directory, 'groundtruth.txt'), delimiter=',', dtype=float))


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
        three-tuple of aggregate metrics, distance_threshold, and number of videos in benchmark.
    """
    if not args.save_dir:
        logging.info("If running benchmark, you must also provide a directory to save results to via the " \
                     "--save_dir argument")
        sys.exit(0)

    if os.path.exists(args.save_dir):
        if not os.path.isdir(args.save_dir):
            logging.info("The path name {0} belongs to a file! aborting.".format(args.save_dir))
            sys.exit(0)
    else:
        os.mkdir(args.save_dir)

    benchmark_sequences = os.listdir(args.sequence_source)
    distance_threshold = load_params(os.path.join('siamfc-params', 'evaluation.json'))['dist_threshold']
    detection_algo = detector.SiamFC() if args.detection_algo == NEURAL_NETWORK else detector.CmtDetector()
    for i, sequence in enumerate(benchmark_sequences):
        logging.info("Running tracking for video {0} of {1}.".format(i + 1, len(benchmark_sequences)))
        try:
            results_filename = sequence + '_results.p'
            results_path = os.path.join(args.save_dir, results_filename)
            if not os.path.exists(results_path):
                sequence_path = os.path.join(args.sequence_source, sequence)
                results, distance_threshold = evaluate_detector_on_sequence(detection_algo, sequence_path, args.visualize, args.preview)
                results.save(results_path)
            else:
                logging.info("Tracking results for sequence {0} already exits. Skipping.".format(sequence))
        except Exception as e:
            logging.warn("Sequence {0} failed with the following exception: {1}".format(sequence, e))


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
    args = get_cli_args()
    log_level = logging.DEBUG if args.db else logging.INFO
    logging.basicConfig(level=log_level)

    alt_run()

    # if args.benchmark:
    #     logging.info("Running benchmark in directory {0} with {1} videos".format(args.sequence_source, len(os.listdir(args.sequence_source))))
    #     run_benchmark(args)
    # else:
    #     logging.info("Tracking object on source {0}".format(args.sequence_source))
    #     run_results = run_single_session(args)
    #     print_metrics(*run_results)


if __name__ == '__main__':
    main()
