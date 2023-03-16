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

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print('File does not exist')
        exit(1)

    # if file size is 0bytes
    if os.path.getsize(csv_path) == 0:
        print('File is empty')
        exit(1)

    file_dir, filename = os.path.split(csv_path)
    filename = filename.split('.')[0]  # drop the suffix/es

    df = pd.read_csv(csv_path)

    # cast data to int
    # t_int = df['time'].astype(int, errors='ignore')
    # det_int = df['num_persons_detected'].astype(int, errors='ignore')
    # mov_int = df['num_moving_detected'].astype(int, errors='ignore')

    ''' Persons (general) '''
    # Plot x=t, y=number of persons detected
    if False:
        # line plot - too messy with noisy data
        plt.plot(df['time'], df['num_persons_detected'])
    else:
        # scatter plot with small points
        plt.scatter(df['time'], df['num_persons_detected'], s=2, color='blue')
        # smoothed plot
        plt.plot(df['time'], df['num_persons_detected'].rolling(window=10).mean(), color='green')

    ''' Moving Persons '''
    plt.scatter(df['time'], df['num_moving_detected'], s=2, color='red')
    plt.plot(df['time'], df['num_moving_detected'].rolling(window=10).mean(), color='orange')

    plt.xlabel('time [ticks]')
    plt.ylabel('persons detected')
    plt.ylim(0)
    plt.xlim(0)
    plt.legend(['persons', 'persons mean (10)', 'moving persons', 'moving mean (10)'])
    plt.suptitle('Number of persons detected over time')
    plt.title(filename)
    # hide top and right axis
    plt.gca().spines['top'].set_visible(False)
    plt.gca().spines['right'].set_visible(False)

    plt.tight_layout()
    # horizontal major grid lines
    plt.grid(axis='y', which='major', linestyle='-', linewidth='0.5', color='grey')


    # save plot
    output_path = os.path.join(file_dir, filename + '.png')

    # warn about overwriting
    if (os.path.exists(output_path)):
        print(f'WARNING: overwriting {output_path}')

    plt.savefig(output_path)
    print(f'saved plot to {output_path}')

    # show plot interactively - unused
    # plt.show()
