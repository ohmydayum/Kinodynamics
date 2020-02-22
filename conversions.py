from arr2_epec_seg_ex import Point_2, Polygon_2

def point_2_to_xy(p):
  return (p.x().to_double(), p.y().to_double())

def xy_to_point_2(x, y):
  return Point_2(x, y)

def coords_list_to_polygon_2(lst):
  lst0 = []
  for i in range(len(lst)//2):
    lst0.append(Point_2(lst[2*i], lst[2*i+1]))
  p = Polygon_2(lst0)
  if p.is_clockwise_oriented(): p.reverse_orientation()
  return p

def tuples_list_to_polygon_2(lst):
  lst0 = []
  for tuple in lst:
    lst0.append(Point_2(tuple[0], tuple[1]))
  p = Polygon_2(lst0)
  if p.is_clockwise_oriented(): p.reverse_orientation()
  return p

def polygon_2_to_tuples_list(polygon):
  lst = [(p.x().to_double(), p.y().to_double()) for p in polygon.vertices()]
  return lst