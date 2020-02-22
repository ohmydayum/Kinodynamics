from arr2_epec_seg_ex import *
from ms_polygon_segment import *

def do_intersect(p1, p2, source, target):
  #fix p1 and move p2 relatively
  ps = Polygon_set_2()
  p1 =  minkowski_sum_polygon_point(p1, source[0])
  ps.insert(p1)
  s = source[1]
  t = target[1] + Vector_2(target[0], source[0])
  if s == t:
    ms = minkowski_sum_polygon_point(p2, s)
  else:
    segment = Segment_2(s, t)
    ms = minkowski_sum_polygon_segment(p2, segment)
  return ps.do_intersect(ms)
