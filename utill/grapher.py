from matplotlib import pyplot as plt

import csv
import os


def readDir(path):
    """
    :param path: Dictionary Path
    :return: type(List)
    """
    if os.path.exists(path):
        files = []
        file_names = []
        _files = os.listdir(path)
        for file in _files:
            if 'csv' in file.split('.'):
                file_names.append(f'{file}')
                files.append(f'{path}/{file}')

        if len(files) == 0:
            raise FileNotFoundError

        csv_datas = []

        for csv_file in files:
            csv_lines = []
            f = open(csv_file, 'r')
            rdr = csv.reader(f)
            for line in rdr:
                csv_lines.append(line)

            csv_datas.append(csv_lines)

        return csv_datas, file_names

    else:
        raise FileNotFoundError

def readTimeData(csv_datas):
    if len(csv_datas) == 0:
        raise IOError

    data_rows = []
    for csv_data in csv_datas:
        time_rows = []
        for csv_rows in csv_data:
            time_data = []
            for string in csv_rows:
                if len(string.split('.')) > 1:
                    time_d = float(string)
                else:
                    time_d = int(string)

                time_data.append(time_d)
            time_rows.append(time_data)
        data_rows.append(time_rows)
    return data_rows


def graphData(data_rows, name):
    for idx, data in enumerate(data_rows):
        time_idx = []
        time_dep = []
        time_n = []


        for time_data in data:
            t_i = time_data[0]
            t_d = 1 / time_data[1]
            t_n = 1 / time_data[2]
            time_idx.append(t_i)
            time_dep.append(t_d)
            time_n.append(t_n)

        # plt.bar(time_idx, time_dep, 1)
        plt.grid()
        colors = ["tab:red" if val < 100 else "tab:blue" for val in time_n]
        plt.title(name[idx])
        plt.ylabel("Hz")
        plt.xlabel("interval(s)")
        plt.bar(time_idx, time_n, width=1, color=colors)
        plt.show()



data, name = readDir('../record')
time_data = readTimeData(data)
graphData(time_data, name)