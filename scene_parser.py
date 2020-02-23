def read_floats_line(f):
    return [float(e) for e in f.readline().split()]


def parse_scene_file_path(path):
    with open(path, "rb") as f:
        raw_lines = f.readlines()
    lines = [[float(num) for num in line.split()] for line in raw_lines]
    robots_count = int(lines[0][0])
    robots_goals = lines[1:robots_count+1]
    robots = lines[robots_count+1:2*robots_count+1]
    obstacles = lines[2*robots_count+1:]
    return robots_count, robots_goals, robots, obstacles
