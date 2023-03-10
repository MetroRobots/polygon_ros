# Representation for a non-simple polygon, i.e. one with holes
polygon_msgs/Polygon2D outer    # The outer perimeter
polygon_msgs/Polygon2D[] inner  # The perimeter of any inner holes
