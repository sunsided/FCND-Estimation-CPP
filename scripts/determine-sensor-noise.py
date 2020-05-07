#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import argparse
import numpy as np
from collections import defaultdict
from typing import Dict, List


def noise_stats_from_csv(file: str):
    reader = csv.DictReader(file)
    print('Found field names:')
    print('\n'.join(f'- {field.strip()}' for field in reader.fieldnames))

    series = defaultdict(list)  # type: Dict[str, List[float]]
    for row in reader:
        for key, value in row.items():
            try:
                series[key].append(float(value))
            except ValueError:
                pass

    print('\nStatistics:')
    for key, values in series.items():
        min_, max_ = np.min(values), np.max(values)
        mean_ = np.mean(values)
        std_ = np.std(values)
        std_err = std_ / np.sqrt(len(values))
        print(f'- Series:     {key.strip()}')
        print(f'  Count:      {len(values)}')
        print(f'  Min:        {min_}')
        print(f'  Max:        {max_}')
        print(f'  Mean:       {mean_}')
        print(f'  Std. Error: {std_err}')
        print(f'  Std. Dev:   {std_}')


def __parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('file', metavar='CSV_FILE', type=argparse.FileType('r'), help='The CSV file to evaluate')
    return parser.parse_args()


def __main():
    args = __parse_args()
    noise_stats_from_csv(args.file)


if __name__ == '__main__':
    __main()
