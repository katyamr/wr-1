# This is a CadQuery script template
# Add your script code below
import cadquery as cq
import math

board_length = 65
board_width = 30
board_thickness = 1.6
board_clearance = 0.6
board_corner_r = 4
fillet_r = 3

# USB-UART board
usb_uart_board_length = 21
usb_uart_board_width = 16

# rim to avoid board movement
usb_uart_board_rim_width = 2

# basement for USB-UART board
usb_uart_base_length = usb_uart_board_length + usb_uart_board_rim_width
usb_uart_base_width = usb_uart_board_width + 2 * usb_uart_board_rim_width

box_inner_height = 12
box_thickness = 2

base_stand_d = 6
screw_cup_hole_d = 5.5
base_edge_height = 2

base_stand_h = 2

cover_stand_d = base_stand_d
cover_stand_d1 = 8
cover_edge_height = 1
cover_clearance = 0.2

stand_x_step = 55
stand_y_step = 25

ant_hole_z = 0 # from center line
ant_hole_y = 0 # from center line

box_inner_length = board_length + 2 * board_clearance
box_inner_width = board_width + 2 * board_clearance

box_outer_length = box_inner_length + 2 * box_thickness
box_outer_width = box_inner_width + 2 * box_thickness
box_outer_height = box_inner_height + box_thickness

# 2.6 x 12 mm screw (d=4.0 mm head)
# base_stand_hole_d = 3.0
# cover_stand_hole_d = 2.5

# 2.3 x 12 mm screw (d=3.6 mm head)
base_stand_hole_d = 2.6
cover_stand_hole_d = 2.2

cover_stand_h = box_inner_height + 1 - base_stand_h - 0.3

# because of (3mm) fillet which rises face edge above
# face height is (box_outer_height - fillet_r)
# face edge is fillet_r above 0
board_top_shift = - (box_outer_height - fillet_r) / 2 - fillet_r + (box_thickness + base_stand_h + board_thickness)

usb_width = 8
usb_height = 3

if 'usb_y_shift' not in locals():
    usb_y_shift = 0

ant_hole_d = 7

usb_z_shift = box_thickness + base_stand_h + board_thickness + usb_height / 2

# New method to render script results using the CadQuery Gateway Interface
# Use the following to render your model with grey RGB and no transparency
# show_object(result, options={"rgba":(204, 204, 204, 0.0)})

stand_centers = [
      (-stand_x_step / 2, -stand_y_step / 2),
      ( stand_x_step / 2, -stand_y_step / 2),
      ( stand_x_step / 2,  stand_y_step / 2),
      (-stand_x_step / 2,  stand_y_step / 2) ]

# make the box base
base = cq.Workplane("XY").rect(box_outer_length, box_outer_width) \
        .extrude(box_outer_height) \
        .edges("|Z").fillet(board_corner_r + board_clearance + box_thickness) \
        .faces("<Z").fillet(fillet_r)

# turn box to shell
base = base.faces(">Z").shell(-box_thickness)

# extrude base stands
base = base.faces("<Z[-2]").workplane(centerOption="CenterOfBoundBox") \
        .pushPoints(stand_centers) \
        .circle(base_stand_d / 2).extrude(base_stand_h)

# extrude USB-UART board basement
base = base.faces("<Z[-2]").workplane(centerOption="CenterOfBoundBox") \
        .center(-(board_length + board_clearance * 2) / 2 + usb_uart_base_length / 2, 0) \
        .rect(usb_uart_base_length, usb_uart_base_width) \
        .extrude(base_stand_h + board_thickness)

# deepen the USB-UART board into basement
base = base.faces("<Z[-4]").workplane(centerOption="CenterOfBoundBox") \
        .center(-1, 0) \
        .rect(usb_uart_board_length, usb_uart_board_width) \
        .cutBlind(-board_thickness)

# drill holes for connector legs
base = base.faces("<Z[-3]").workplane(centerOption="CenterOfBoundBox") \
        .center(9, 0) \
        .rarray(1, 2.54, 1, 6, True) \
        .hole(2.54, 2)


# making holes in base
base = base.faces("<Z").workplane(centerOption="CenterOfBoundBox") \
        .pushPoints(stand_centers) \
        .cboreHole(base_stand_hole_d, screw_cup_hole_d, 2)

# making antenna hole
base = base.faces(">X").workplane(centerOption="CenterOfBoundBox") \
        .center(ant_hole_y, board_top_shift * 0.0 + ant_hole_z) \
        .hole(ant_hole_d, box_thickness)

# making micro-USB hole shape
usb = cq.Workplane("YZ").rect(usb_width, usb_height) \
        .extrude(-box_thickness) \
        .edges("|X").fillet(1) \
        .translate((-box_inner_length / 2, usb_y_shift, usb_z_shift))

# cut the hole in base
base = base.cut(usb)


# make the cover
cover = cq.Workplane("XY").rect(box_outer_length, box_outer_width) \
        .extrude(fillet_r + 0.1) \
        .edges("|Z").fillet(board_corner_r + board_clearance + box_thickness) \
        .faces(">Z").fillet(fillet_r) \
        .faces("<Z").shell(-box_thickness) \
        .translate((0, 0, box_thickness - fillet_r))

# USB-UART board cover
cover = cover.faces(">Z[-2]").workplane(centerOption="CenterOfBoundBox") \
        .center(-(board_length + board_clearance * 2) / 2 + usb_uart_base_length / 2, 0) \
        .rect(10, usb_uart_base_width) \
        .extrude(cover_stand_h - board_thickness - 1) \
        .edges("|Z").fillet(2)

# cover stands
stands = cq.Workplane("XY").rect(box_inner_length + 2 * cover_stand_d1, box_inner_width + 2 * cover_stand_d1) \
        .extrude(1)

# cover rim
cover_rim = cq.Workplane("XY").rect(box_inner_length - 2 * cover_clearance, box_inner_width - 2 * cover_clearance) \
        .extrude(fillet_r) \
        .edges("|Z").fillet(board_corner_r + board_clearance - cover_clearance) \
        .faces("<Z").shell(-1) \
        .translate((0, 0, 1 - fillet_r))

for p in stand_centers:
    stands = stands.faces(">Z[-2]").workplane(centerOption="CenterOfBoundBox") \
        .center(*p) \
        .circle(cover_stand_d1 / 2) \
        .workplane(offset=cover_stand_h) \
        .circle(cover_stand_d / 2) \
        .loft(combine=True)

stands = stands.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
        .pushPoints(stand_centers) \
        .hole(cover_stand_hole_d)

# box inner shape
i = cq.Workplane("XY") \
        .rect(box_inner_length - 2 * cover_clearance, box_inner_width - 2 * cover_clearance) \
        .extrude(-cover_stand_h) \
        .edges("|Z").fillet(board_corner_r + board_clearance - cover_clearance)

stands = stands.intersect(i)

cover = cover.union(stands).union(cover_rim)

# move the cover up
#cover = cover.translate((0, 0, box_inner_height * 2))

# holes for LED in the top
cover = cover.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
        .center(-(board_length + board_clearance * 2) / 2 + 7, 5) \
        .hole(2)

cover = cover.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
        .center(-(board_length + board_clearance * 2) / 2 + 11, 5) \
        .hole(2)

cover = cover.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
        .center(-(board_length + board_clearance * 2) / 2 + 12.5, -5) \
        .hole(2)

# "open up" the box
cover = cover.rotate((0, 0, 0), (1, 0, 0), 180) \
    .translate((0, box_outer_width + 5, box_thickness + 0.1))

if __name__ == '__cqgi__':
    show_object(cover)
    show_object(base)
#    show_object(logo)
else:
    from cadquery import exporters

    box = cover.union(base)

    name = 'box_%dx%d' % (midi_sock_count, midi_sock_count)
    exporters.export(box, name + '.step')
    exporters.export(box, name + '.stl')
    exporters.export(box, name + '.vrml')
