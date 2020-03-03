from arr2_epec_seg_ex import Point_2


def read_floats_line(f):
    return [float(e) for e in f.readline().split()]


def split_list_to_couples(l):
    return [Point_2(l[2*i], l[2*i+1])
            for i in range(int(len(l)/2))]


def parse_scene_file_path(path):
    with open(path, "rb") as f:
        raw_lines = f.readlines()
    lines = [[float(num) for num in line.split()] for line in raw_lines]
    robots_count = int(lines[0][0])
    robots_goals = [split_list_to_couples(l)[0] for l in lines[1:robots_count+1]]
    robots = [split_list_to_couples(l) for l in lines[robots_count+1:2*robots_count+1]]
    obstacles = [split_list_to_couples(l) for l in lines[2*robots_count+1:]]
    return robots_count, robots_goals, robots, obstacles
