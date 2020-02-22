import re
f = open("geogebra-export.txt", "r")
out = open("obstacle_lines_for_scene.txt", "w")
for line in f:
    polygon = []
    if "\pspolygon" not in line:
        continue
    lst = re.split("[()]",line)
    for item in lst:
        vals =item.split(",")
        if len(vals) == 2:
            polygon.extend([vals[0], vals[1]])
    out.write(str(len(polygon)//2) + " " + " ".join(polygon) +
"\n")
out.close()
f.close()
