#!/usr/bin/python3

from dataset import Dataset
import os
import glob
import rospkg


def main():
    dataset_path=f'{os.environ["DORA"]}/rgbd-dataset'
    image_filenames = glob.glob(dataset_path + '/*/*/*_crop.png')
    dataset=Dataset(image_filenames)

if __name__ == '__main__':
    main()
