import cadquery as cq
import math

base_d = 87
base_thickness = 3

fill_line_width = 1.5
fill_line_step = 8

# make the base
base_shape = cq.Workplane("XY") \
        .circle(base_d / 2) \
        .extrude(base_thickness) \

base = base_shape.translate((0,0,0))

base = base.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
        .circle(base_d / 2 - base_thickness) \
        .cutBlind(-base_thickness)

fill_line = cq.Workplane("XY").rect(base_d, fill_line_width) \
        .extrude(base_thickness)

y = -base_d / 2
while y <= base_d / 2:
    y += fill_line_step
    base = base.union(fill_line.translate((0, y, 0)))

fill_line = cq.Workplane("XY").rect(fill_line_width, base_d) \
        .extrude(base_thickness)

x = -base_d / 2
while x <= base_d / 2:
    x += fill_line_step
    base = base.union(fill_line.translate((x, 0, 0)))

base = base.intersect(base_shape)

base = base.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
        .center(-10, -20).circle(10 / 5) \
        .center(20, 10).rect(14, 9) \
        .cutBlind(-base_thickness)

if __name__ == '__cqgi__':
    show_object(base)
