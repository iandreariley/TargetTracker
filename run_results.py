import os
import cv2
import evaluation
import argparse
import pickle


def get_args():
    parser = argparse.ArgumentParser(description="Visualize results of previous tracking session.")
    parser.add_argument("results_file", help="Picked TrackingResults object that contains the tracking results.")
    parser.add_argument("--video_dir", help="Video dir if different than stored", default="")
    return parser.parse_args()


def draw_bbox(image, bbox):
    top_left = bbox[0], bbox[1]
    bottom_right = bbox[0] + bbox[2], bbox[1] + bbox[3]
    top_right = (bottom_right[0], top_left[1])
    bottom_left = (top_left[0], bottom_right[1])

    # draw bounding box.
    cv2.line(image, top_left, top_right, (255, 0, 0), 4)
    cv2.line(image, top_right, bottom_right, (255, 0, 0), 4)
    cv2.line(image, bottom_right, bottom_left, (255, 0, 0), 4)
    cv2.line(image, bottom_left, top_left, (255, 0, 0), 4)


def resolve_directory(image_file, args):
    if not args.video_dir:
        return image_file

    path_components = image_file.split('/')
    filename = path_components[-2:]
    return os.path.join(args.video_dir, *filename)


def main(args):
    with open(args.results_file, 'rb') as results_file:
        results = pickle.load(results_file)

    for image_file, bbox in results.predictions.items():
        image_file = resolve_directory(image_file, args)
        image = cv2.imread(image_file)
        draw_bbox(image, bbox)
        cv2.imshow('preview', image)
        cv2.waitKey(10)


if __name__ == '__main__':
    args = get_args()
    main(args)
