import logging
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

if __name__ == '__main__':
    # read csv given by argument
    # first argument is the path to the csv file
    if not len(sys.argv) == 2:
        print('Usage: python evaluation.py <path to csv file>')
        exit(1)
    
    if not os.path.exists(sys.argv[1]):
        print('File does not exist')
        exit(1)

    # if file size is 0bytes
    if os.path.getsize(sys.argv[1]) == 0:
        print('File is empty')
        exit(1)

    file_dir, filename = os.path.split(sys.argv[1])

    df = pd.read_csv(sys.argv[1], header=None, names=['t', 'number_of_persons'], encoding=None, on_bad_lines='warn', encoding_errors='replace')

    # Plot x=t, y=number of persons detected
    plt.plot(df['number_of_persons'])
    plt.xlabel('time [ticks]')
    plt.ylabel('number of persons detected')
    plt.title('Number of persons detected over time')
    
    # remove suffixes from filename
    filename = filename.split('.')[0]

    output_path = os.path.join(file_dir, filename + '.png')

    # save plot
    plt.savefig(output_path)
    print(f'saved plot to {output_path}')

