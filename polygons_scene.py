from arr2_epec_seg_ex import *
import math
import importlib
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF
import read_input
from conversions import point_2_to_xy, tuples_list_to_polygon_2, polygon_2_to_tuples_list
import ms_polygon_segment, linear_path_intersection_test

offset = -Vector_2(FT(0), FT(0))
colors = [Qt.magenta, Qt.green, Qt.red, Qt.blue, Qt.yellow]

class Polygons_scene():
  def __init__(self):
    self.number_of_robots = None
    self.robots = []
    self.obstacles = []
    self.gui_robots = []
    self.gui_obstacles = []
    self.destinations = []
    self.gui_destinations = []
    self.path = []
    self.all_edges = []

  def draw_scene(self):
    gui.clear_scene()
    self.gui_robots = []
    self.gui_robots.append(gui.add_polygon([point_2_to_xy(p) for p in self.robots[0]], Qt.yellow))
    self.gui_obstacles = []
    for obstacle in self.obstacles:
      self.gui_obstacles.append(gui.add_polygon([point_2_to_xy(p) for p in obstacle], Qt.darkGray))
    self.gui_destinations = []
    for i in range(len(self.destinations)):
        self.gui_destinations.append(gui.add_disc(0.05, *point_2_to_xy(self.destinations[i]), Qt.green))

  def load_scene(self, filename):
    scene = read_input.read_polygon_scene(filename)
    self.number_of_robots = scene[0]
    self.robots = []
    self.obstacles = []
    self.path = []
    self.gui_robots = []
    self.gui_obstacles = []
    self.destinations = []
    self.gui_destinations = []
    destinations = []
    for i in range(self.number_of_robots):
      destinations.append(scene[i+1])
    self.set_destinations(destinations)
    for i in range(self.number_of_robots):
      self.robots.append(scene[i+self.number_of_robots + 1])
    self.obstacles = []
    for i in range(self.number_of_robots*2 + 1, len(scene)):
      self.obstacles.append(scene[i])
    gui.clear_queue()
    self.draw_scene()

  def set_destinations(self, destinations):
    self.destinations = destinations
    for i in range(len(self.gui_destinations)):
      if self.gui_destinations[i] != None:
        self.gui_destinations[i].pos = QPointF(*point_2_to_xy(destinations[i]))

  def set_up_animation(self):
    self.draw_scene()
    animations = []
    if OPTIONS['re/draw']:
        for e in self.all_edges:
            for ee in e.sampled_edges:
                gui.add_segment(*(ee.previous_edge.target), *(ee.target), Qt.lightGray)
    if len(self.path) == 0:
      return
    if len(self.path) == 1:
      for i in range(self.number_of_robots):
        start = point_2_to_xy(self.path[0][i] + offset)
        animations.append(gui.linear_translation_animation(self.gui_robots[i], *start, *start))
      anim = gui.parallel_animation(*animations)
      gui.queue_animation(anim)
    else:
      for i in range(len(self.path) - 1):
        animations = []
        start = point_2_to_xy(self.path[i])
        end = point_2_to_xy(self.path[i+1])
        s = gui.add_segment(*start, *end, Qt.red)
        start = point_2_to_xy(self.path[i])
        end = point_2_to_xy(self.path[i + 1])
        s.line.setZValue(2)
        gui.add_disc(0.02, *end, fill_color=Qt.blue)
        animations.append(gui.linear_translation_animation(self.gui_robots[0], *start, *end))
        anim = gui.parallel_animation(*animations)
        gui.queue_animation(anim)

  def is_path_valid(self):
    robot_intersection_check = True
    if self.path == None: return False
    if len(self.path) == 0: return False
    robot_polygons = []
    path_polygons = []
    if len(self.path) > 1:
      for i in range(len(self.path) - 1):
        source = []
        target = []
        for j in range(self.number_of_robots):
          robot_polygons.append(Polygon_2(self.robots[j]))
          source.append(self.path[i][j])
          target.append(self.path[i+1][j])
          if source[j] != target[j]:
            s = Segment_2(source[j], target[j])
            pwh = ms_polygon_segment.minkowski_sum_polygon_segment(robot_polygons[j], s)
          else:
            pwh = ms_polygon_segment.minkowski_sum_polygon_point(robot_polygons[j], source[j])
          path_polygons.append(pwh)
        # check that the robots don't intersect each other while performing the current step
        for j in range(self.number_of_robots):
          for k in range(self.number_of_robots):
            if(j!=k):
              if linear_path_intersection_test.do_intersect(robot_polygons[j], robot_polygons[k], source, target): robot_intersection_check = False

    obstacle_polygons = []
    for obs in self.obstacles:
      p = Polygon_2(obs)
      obstacle_polygons.append(p)

    path_set = Polygon_set_2()
    path_set.join_polygons_with_holes(path_polygons)
    obstacles_set = Polygon_set_2()
    obstacles_set.join_polygons(obstacle_polygons)

    lst = []
    path_set.polygons_with_holes(lst)
    for pwh in lst:
      p = pwh.outer_boundary()
      lst = polygon_2_to_tuples_list(p)
      gui.add_polygon(lst, Qt.lightGray).polygon.setZValue(-3)
      for p in pwh.holes():
        lst = polygon_2_to_tuples_list(p)
        gui.add_polygon(lst, Qt.white).polygon.setZValue(-2)


    # check that the origin matches the first point in the path
    origin_check = True
    for i in range(self.number_of_robots):
      if self.robots[i][0] - offset != self.path[0][i]: origin_check = False
    # check that the destination matches the last point in the path
    destination_check = True
    for i in range(self.number_of_robots):
      if self.destinations[i] != self.path[-1][i]: destination_check = False
    # check that there are no collisions with obstacles
    obstacle_intersection_check = True if not path_set.do_intersect(obstacles_set) else False
    res = (origin_check and destination_check and obstacle_intersection_check and robot_intersection_check)
    print("Valid path: ", res)
    if(origin_check == False):
      print("Origin mismatch")
    if(destination_check == False):
      print("Destination mismatch")
    if obstacle_intersection_check == False:
      print("Movement along path intersects with obstacles")
    if robot_intersection_check == False:
      print("The robots intersect each other")
    return res

def set_up_scene():
  gui.clear_scene()
  scene_file = gui.get_field(0)
  ps.load_scene(scene_file)
  print("loaded scene from", scene_file)

def generate_path():
  ps.path = []
  ps.all_edges = []
  path_name = gui.get_field(1)
  gp = importlib.import_module(path_name)
  gp.generate_path(ps.path, ps.robots, ps.obstacles, ps.destinations, ps.all_edges, OPTIONS)
  gui.clear_queue()
  ps.set_up_animation()
  print("Generated path via", path_name + ".generate_path")

# def load_path():
#   ps.path = []
#   gui.clear_queue()
#   path_name = gui.get_field(2)
#   read_input.load_path(ps.path, path_name, ps.number_of_robots)
#   print("Loaded path from", path_name)
#   ps.set_up_animation()

def is_path_valid():
  ps.is_path_valid()

def animate_path():
  gui.play_queue()


def set_option(n, key, options):
  def set_specific_key_to_n():
    options[key] = float(gui.get_field(n))
  return set_specific_key_to_n


def ez_option_text(options, gui, n, key, value):
  gui.set_button_text(n, key)
  gui.set_field(n, str(value))
  gui.set_logic(n, lambda x: options.update({key: float(gui.get_field(n))}))
  options[key] = float(value)


def ez_option_button(options, gui, n, key, value):
    def update_button():
        options.update({key: not options.get(key, value)})
        gui.turn_button_on(n, options[key])
        ps.set_up_animation()
    gui.set_logic(n, update_button)
    gui.set_button_text(n, key)
    gui.turn_button_on(n, value)
    options[key] = float(value)


if __name__ == "__main__":
  import sys
  app = QtWidgets.QApplication(sys.argv)
  gui = GUI()
  OPTIONS = {}
  ps = Polygons_scene()
  gui.set_program_name("Kinodynamics- Dor Israeli")
  gui.set_button_text(0, "Load scene")
  gui.set_field(0, "scene0")
  gui.set_logic(0, set_up_scene)
  gui.set_button_text(1, "Generate path")
  gui.set_field(1, "rrt")
  gui.set_logic(1, generate_path)
  ez_option_text(OPTIONS, gui, 2, "K", 1000)
  ez_option_text(OPTIONS, gui, 3, "dt", 1)
  ez_option_text(OPTIONS, gui, 4, "g", -0.1)
  ez_option_text(OPTIONS, gui, 5, "epsilon", 1)
  ez_option_text(OPTIONS, gui, 6, "thrust", 1)
  ez_option_text(OPTIONS, gui, 7, "mass", 1)
  ez_option_button(OPTIONS, gui, 8, "re/draw", True)
  ez_option_button(OPTIONS, gui, 9, "two-sided", True)
  gui.MainWindow.show()
  sys.exit(app.exec_())