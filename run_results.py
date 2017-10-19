import os
import cv2
import evaluation
import argparse
import pickle


PRED_COLOR = (0, 0, 255)
GT_COLOR = (255, 0, 0)


def get_args():
    parser = argparse.ArgumentParser(description="Visualize results of previous tracking session.")
    parser.add_argument("results_file", help="Picked TrackingResults object that contains the tracking results.")
    parser.add_argument("--video_dir", help="Video dir if different than stored", default="")
    parser.add_argument("--show_gt", action="store_true", help="Show ground truth bounding box.")
    parser.add_argument("--speed", type=int, default=10, help="duration (in ms) of each frame")
    return parser.parse_args()


def draw_bbox(image, bbox, color):
    top_left = intify((bbox[0], bbox[1]))
    bottom_right = intify((bbox[0] + bbox[2], bbox[1] + bbox[3]))
    top_right = (bottom_right[0], top_left[1])
    bottom_left = (top_left[0], bottom_right[1])

    # draw bounding box.
    cv2.line(image, top_left, top_right, color, 4)
    cv2.line(image, top_right, bottom_right, color, 4)
    cv2.line(image, bottom_right, bottom_left, color, 4)
    cv2.line(image, bottom_left, top_left, color, 4)


def intify(tup):
    return tuple(map(int, tup))

def resolve_directory(image_file, args):
    if not args.video_dir:
        return image_file

    path_components = image_file.split('/')
    filename = path_components[-2:]
    return os.path.join(args.video_dir, *filename)


def main(args):
    with open(args.results_file, 'rb') as results_file:
        results = pickle.load(results_file)

    for i, item in enumerate(results.predictions.items()):
        image_file, bbox = item
        image_file = resolve_directory(image_file, args)
        image = cv2.imread(image_file)
        if args.show_gt:
            draw_bbox(image, map(int, results.ground_truth[i]), GT_COLOR)
        draw_bbox(image, bbox, PRED_COLOR)
        cv2.imshow('preview', image)
        cv2.waitKey(10)


if __name__ == '__main__':
    args = get_args()
    main(args)
