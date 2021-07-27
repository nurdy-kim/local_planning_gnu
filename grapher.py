from matplotlib import pyplot as plt

import csv
import os
import numpy

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
                elif len(string.split('.')) <= 1 and string.isnumeric():
                    time_d = int(string)
                else:
                    time_d = str(string)

                time_data.append(time_d)
            time_rows.append(time_data)
        data_rows.append(time_rows)
    return data_rows


def graphData(data_rows, name):
    for idx, data in enumerate(data_rows):
        time_idx = []
        time_dep = []
        time_n = []
        time_gp_n = []
        time_lp_n = []

        for time_data in data:
            if len(time_data) < 3:
                continue
            if len(time_data) == 3:
                # 통합
                t_i = time_data[0]
                t_d = time_data[1]
                t_n = time_data[2] * 1000
                time_idx.append(t_i)
                time_dep.append(t_d)
                time_n.append(t_n)

            if len(time_data) > 3:
                # GP, LP 분리
                t_i = time_data[0]
                t_d = time_data[1]
                t_n = time_data[2] * 1000
                t_t = time_data[3]

                time_idx.append(t_i)
                time_dep.append(t_d)
                if t_t == "gp":
                    # time_lp_n.append(0)
                    time_gp_n.append(t_n)
                elif t_t == "lp":
                    time_lp_n.append(t_n)
                    # time_gp_n.append(0)

        print("---------------------------")
        print(f"Result of {name[idx]}")
        # print(f"Intervals: {t_i}")
        if len(time_n) != 0:
            print(f"Mean:   {round(numpy.mean(time_n), 6)}")

            # print(f"Median: {round(numpy.median(time_n), 6)}")
            # print(f"Max:    {round(max(time_n), 6)}")
            # print(f"Min:    {round(min(time_n), 6)}")
            # print(f"Total Time: {round(t_d, 6)}")

            plt.grid()
            # colors = ["tab:red" if val < 100 else "tab:blue" for val in time_n]
            plt.title(name[idx])
            plt.ylabel("milliseconds")
            plt.xlabel("interval(s)")
            plt.bar(time_idx, time_n, width=1)
            plt.show()

        else:
            print(f"Mean GP:   {round(numpy.mean(time_gp_n), 6)}")
            print(f"Mean LP:   {round(numpy.mean(time_lp_n), 6)}")
            # print(f"Median: {round(numpy.median(time_n), 6)}")
            # print(f"Max:    {round(max(time_n), 6)}")
            # print(f"Min:    {round(min(time_n), 6)}")
            # print(f"Total Time: {round(t_d, 6)}")

            # plt.grid()
            # colors = ["tab:red" if val < 100 else "tab:blue" for val in time_n]
            # plt.title(name[idx])
            # plt.ylabel("millisecond(s)")
            # plt.xlabel("interval(s)")
            # plt.bar(time_idx, time_gp_n, width=1, color="tab:blue")
            # plt.bar(time_idx, time_lp_n, width=1, color="tab:red")
            # plt.show()










data, name = readDir('record/vegas_obs')
time_data = readTimeData(data)
graphData(time_data, name)