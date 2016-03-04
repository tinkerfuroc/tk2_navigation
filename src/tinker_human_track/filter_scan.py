import sys
from numpy import array, load, mean, std
from numpy import inf
from math import sqrt, acos, pi
import matplotlib.pyplot as plt

def get_distance(x1, y1, x2, y2):
    return sqrt((x1-x2) ** 2 + (y1-y2) ** 2)

def near_enough(x1, y1, x2, y2):
    return get_distance(x1, y1, x2, y2) < 0.1

def get_angle(start_x, start_y, end_x, end_y, x, y):
    x1 = x - start_x
    x2 = x - end_x
    y1 = y - start_y
    y2 = y - end_y
    dot_product = x1 * x2 + y1 * y2
    return acos(dot_product / (get_distance(x1, y1, 0, 0) * get_distance(x2, y2, 0, 0)))

def toward_weight(segment):
    start_x = segment[0][0]
    start_y = segment[0][1]
    end_x = segment[-1][0]
    end_y = segment[-1][1]
    count_towards = 0
    count_backwards = 0
    for x, y in segment[1: len(segment) -1]:
        if ((x - start_x) * (y - end_y) - (x - end_x) * (y - start_y)) * (start_x * end_y - end_x * start_y) > 0:
            count_towards += 1
        else:
            count_backwards += 1
    return float(count_towards) / (count_towards + count_backwards)


def mean_circle_angle(segment):
    start_x = segment[0][0]
    start_y = segment[0][1]
    end_x = segment[-1][0]
    end_y = segment[-1][1]
    count_towards = 0
    count_backwards = 0
    angles = [get_angle(start_y, start_y, end_x, end_y, x, y) for x, y in segment[1: len(segment) - 1]]
    return mean(array(angles))


def is_valid_segment(segment):
    if len(segment) < 15 or len(segment) > 80:
        return False
    x_diff = segment[0][0] - segment[-1][0]
    y_diff = segment[0][1] - segment[-1][1]
    seg_distance = sqrt(x_diff ** 2 + y_diff ** 2)
    if seg_distance > 0.4:
        return False
    circle_angle = mean_circle_angle(segment)
    if circle_angle < 0.5 or circle_angle > 3:
        return False
    if toward_weight(segment) < 0.8:
        return False
    return True



def init(argv):
    laser_data = load(argv[1])
    laser_data = array([[x, y] for x, y in laser_data if -100 < x < 100 and -100 < y < 100])
    plt.scatter(laser_data[:, 0], laser_data[:, 1])
    plt.axis([-6,6,-6,6])
    plt.show()
    now_segment = []
    last_x = 0
    last_y = 0
    in_segment = False
    final_xs = []
    final_ys = []
    for x, y in laser_data:
        if not in_segment:
            now_segment = []
            now_segment.append([x, y])
            in_segment = True
        else:
            if near_enough(last_x, last_y, x, y):
                now_segment.append([x, y])
            else:
                if is_valid_segment(now_segment):
                    for seg_x, seg_y in now_segment:
                        final_xs.append(seg_x)
                        final_ys.append(seg_y)
                now_segment = [[x,y]]
        last_x = x
        last_y = y
    plt.scatter(final_xs, final_ys)
    plt.axis([-6,6,-6,6])
    plt.show()


if __name__ == '__main__':
    init(sys.argv)
