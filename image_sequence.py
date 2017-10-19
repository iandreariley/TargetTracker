import os
import scipy.ndimage as ndimage


class DirectorySequence:
    """Iterable image sequence drawn from a directory of image files."""

    def __init__(self, sequence_directory, ordering=None, image_format='jpg'):
        """Initialize sequence from directory.

        Args:
            sequence_directory (str): path to directory that contains image sequence (as individual files).
            ordering (fn): Optional. A function to be passed to the `key` argument to the `sorted` built-in function
                which is used to order images by filename. If nothing is passed, all files with an `image_format`
                extension are ordered numerically.
            image_format (str): Optional. The image format of the image files in the sequence. Identified by the file
                extension used for these files (e.g. `jpg`).

        Raises:
            FileNotFoundException: If `sequence_directory` does not exist.
        """

        image_files = filter(lambda filename: filename.endswith(image_format), os.listdir(sequence_directory))

        if ordering is None:
            ordering = self._numerical_value

        self._sequence = map(lambda filename: os.path.join(sequence_directory, filename),
                             sorted(image_files, key=ordering))
        self._index = 0
        self._size = len(self._sequence)

        if len(self._sequence) == 0:
            raise ValueError("Sequence could not be initialized! No files of type '{0}' could be found in directory "
                             "'{1}'".format(image_format, sequence_directory))

    def __iter__(self):
        return self

    def __len__(self):
        return len(self._sequence)

    def next(self):
        if self._index < self._size:
            file_path = self._sequence[self._index]
            img = ndimage.imread(file_path)
            self._index += 1
            return file_path, img
        else:
            self._index = 0
            raise StopIteration()

    @staticmethod
    def _numerical_value(filename):
        """Turn filename (minus extension) into an integer value"""
        return int(filename[:filename.index('.')])
