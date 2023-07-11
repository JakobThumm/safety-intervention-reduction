"""Load in the csv files of multiple runs and summarize them."""

import os
import pandas as pd
import argparse
import numpy as np
from scipy.stats import bootstrap


def create_csv_paths_dict(root_dir):
    """Create a dictionary of csv paths for each environment and algorithm."""
    csv_paths = {}
    for day_dir in os.listdir(root_dir):
        day_path = os.path.join(root_dir, day_dir)
        if not os.path.isdir(day_path):
            continue
        for time_dir in os.listdir(day_path):
            time_path = os.path.join(day_path, time_dir)
            if not os.path.isdir(time_path):
                continue
            for env_dir in os.listdir(time_path):
                env_path = os.path.join(time_path, env_dir)
                if not os.path.isdir(env_path):
                    continue
                for alg_seed_dir in os.listdir(env_path):
                    alg_seed_path = os.path.join(env_path, alg_seed_dir)
                    if not os.path.isdir(alg_seed_path):
                        continue
                    if alg_seed_dir.startswith('run_'):
                        items = alg_seed_dir[len('run_'):].split('_')
                        algorithm = items[0]
                        for item in items[1:-1]:
                            algorithm += '_' + item
                        csv_path = os.path.join(alg_seed_path, 'progress.csv')
                        if env_dir not in csv_paths:
                            csv_paths[env_dir] = {}
                        if algorithm not in csv_paths[env_dir]:
                            csv_paths[env_dir][algorithm] = []
                        csv_paths[env_dir][algorithm].append(csv_path)
    return csv_paths


def create_all_summaries(csv_paths, output_folder, window_size):
    """Create a summary csv for each environment and algorithm."""
    for environment in csv_paths:
        env_folder = os.path.join(output_folder, environment)
        if not os.path.exists(env_folder):
            os.mkdir(env_folder)
        for algorithm in csv_paths[environment]:
            csv_all_seeds = csv_paths[environment][algorithm]
            # output the dataframe to a csv file
            output_df = average_all_seeds(csv_all_seeds, window_size)
            # get name of last folder in path_to_folder
            output_df.to_csv(os.path.join(env_folder, '{}.csv'.format(algorithm)), index=False)


def average_all_seeds(csv_all_seeds, window_size=1):
    """Convert multiple training runs to single csv.

    Take in a path to a folder containing multiple training run seeds,
    and output a csv file with columns
    step, reward_mean, reward_std, cost_mean, cost_std, failsafe_mean, failsafe_std.
    """
    # create an empty list to store the numpy arrays
    steps = None
    value_dict = {
        "Return": [],
        "Cost": [],
        "ShieldActivation": []
    }

    # loop through each subfolder and load the progress.csv file into a pandas dataframe
    for csv_file in csv_all_seeds:
        df = pd.read_csv(csv_file)
        if steps is None:
            steps = df['Diagnostics/CumSteps'].values
        value_dict['Return'].append(df['ReturnAverage'].values)
        value_dict['Cost'].append(df['CostAverage'].values)
        value_dict['ShieldActivation'].append(df['ShieldActivationAverage'].values)

    # concatenate the numpy arrays on a new axis
    for key in value_dict:
        value_dict[key] = np.stack(value_dict[key], axis=0)

    # Make sure the window size is odd
    if window_size % 2 == 0:
        window_size += 1
    half_window = (window_size-1) // 2
    output_value_dict = {}
    length = None
    for key in value_dict:
        if not length:
            length = value_dict[key].shape[1]-2*half_window
        output_value_dict[key + "Average"] = np.zeros(value_dict[key].shape[1]-2*half_window)
        output_value_dict[key + "Std"] = np.zeros(value_dict[key].shape[1]-2*half_window)
        output_value_dict[key + "Bootstrap025"] = np.zeros(value_dict[key].shape[1]-2*half_window)
        output_value_dict[key + "Bootstrap975"] = np.zeros(value_dict[key].shape[1]-2*half_window)
    step = np.zeros(length)
    for i in range(half_window, length+half_window):
        for key in value_dict:
            if window_size == 1:
                output_value_dict[key + "Average"][i-half_window] = np.mean(value_dict[key][:, i])
                output_value_dict[key + "Std"][i-half_window] = np.std(value_dict[key][:, i])
                data = value_dict[key][:, i]
                res = bootstrap(data=data[np.newaxis, :],
                                statistic=np.mean,
                                confidence_level=0.95,
                                axis=0,
                                n_resamples=1000)
                output_value_dict[key + "Bootstrap025"][i-half_window] = res.confidence_interval.low
                output_value_dict[key + "Bootstrap975"][i-half_window] = res.confidence_interval.high
            else:
                output_value_dict[key + "Average"][i-half_window] = np.mean(value_dict[key][:, i-half_window:i+half_window])
                output_value_dict[key + "Std"][i-half_window] = np.std(value_dict[key][:, i-half_window:i+half_window])
                data = np.concatenate(value_dict[key][:, i-half_window:i+half_window], axis=0)
                res = bootstrap(data=data[np.newaxis, :],
                                statistic=np.mean,
                                confidence_level=0.95,
                                axis=0,
                                n_resamples=1000)
                output_value_dict[key + "Bootstrap025"][i-half_window] = res.confidence_interval.low
                output_value_dict[key + "Bootstrap975"][i-half_window] = res.confidence_interval.high
        step[i-half_window] = steps[i]
    output_value_dict['TotalEnvInteracts'] = step
    # value_dict['ShieldActivation'][:, 100]
    # create a new dataframe with the calculated values
    output_df = pd.DataFrame(output_value_dict)
    return output_df


def load_csv(csv_path):
    """Load in the csv file of a single run."""
    df = pd.read_csv(csv_path)
    return df


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Load CSV table')
    parser.add_argument(
        '--window_size',
        required=False,
        type=int,
        help='Window size for smoothing',
        default=1
    )
    args = parser.parse_args()
    this_dir = os.path.dirname(os.path.abspath(__file__))
    csv_paths = create_csv_paths_dict(
        this_dir + '/../data/local/'
    )
    if not os.path.exists(this_dir + '/../data/summaries/'):
        os.mkdir(this_dir + '/../data/summaries/')
    create_all_summaries(csv_paths, this_dir + '/../data/summaries/', args.window_size)
    print("Summary created in data/summaries/")
