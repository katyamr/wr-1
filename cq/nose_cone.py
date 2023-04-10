# This is a CadQuery script template
# Add your script code below
import cadquery as cq
import math

Rout = 91 / 2
Rin = 88 / 2
#Rout = 93 / 2
#Rin = 90.6 / 2
h = 130
x0 = (h ** 2 - Rout ** 2) / (2 * Rout)
th = 1.6
cam_rotate_angle = -45

shield_r = 18
shield_h = 25
shield_x0 = (shield_h ** 2 - shield_r ** 2) / (2 * shield_r)

# make the shield cone
shield_cone = cq.Workplane("YZ") \
      .vLine(shield_h) \
      .radiusArc((shield_r, 0), shield_x0 + shield_r) \
      .close() \
      .revolve() \
      .faces("<Z") \
      .shell(3)

# make the cone
cone = cq.Workplane("YZ") \
      .vLine(h) \
      .radiusArc((Rout, 0), x0 + Rout) \
      .line(Rin - Rout, Rin - Rout) \
      .vLine(-20 + Rout - Rin) \
      .hLine(-Rin) \
      .close() \
      .revolve()

inner_cone = cq.Workplane("YZ") \
      .vLine(h - th) \
      .radiusArc((Rout - th, th), x0 + Rout - th) \
      .hLine(Rin - Rout - 5 + th) \
      .vLine(-20) \
      .hLine(-Rin + 5) \
      .close() \
      .revolve()

inner_edge = cq.Workplane("XY") \
      .rect(th, 2 * Rout).extrude(h + 20) \
      .translate((0, 0, -20))

#cone = cone.union(shield_cone.translate((0, Rout - 8, 10)))

cone = cone.faces("<Z") \
      .workplane(centerOption="CenterOfBoundBox") \
      .hole(Rin * 2 - 10, 3)

cone = cone.faces("<Z").fillet(2)

cam = cq.Workplane("XY") \
      .rect(16, 40) \
      .extrude(16)

cam_shell = cam.translate((0, 0, 0)).faces(">Z").shell(th)

cam = cam.rotate((0, 0, 0), (1, 0, 0), cam_rotate_angle)
cam_shell = cam_shell.rotate((0, 0, 0), (1, 0, 0), cam_rotate_angle)

cam = cam.translate((0, Rout - 10, 5))
cam_shell = cam_shell.translate((0, Rout - 10, 5)) \
      .intersect(inner_cone)

bat = cq.Workplane("XY") \
      .rect(8, 21) \
      .extrude(40) \
      .translate((0, 0, 20))

bat_shell = bat.translate((0, 0, 0)).faces(">Z").shell(th) \
      .cut(bat.translate((-4, 12, 0)))

bat = bat.faces(">Z").workplane(centerOption="CenterOfBoundBox") \
      .rect(8, 21) \
      .workplane(offset=30 / 2) \
      .rect(0.1, 0.1) \
      .loft(combine=True)

inner_cone = inner_cone.cut(inner_edge) \
      .cut(inner_edge.rotate((0, 0, 0), (0, 0, 1), 60)) \
      .cut(inner_edge.rotate((0, 0, 0), (0, 0, 1), -60)) \
      .cut(cam_shell) \
      .cut(bat_shell)

rope_ring = cq.Workplane("YZ") \
      .center(10, -5) \
      .circle(3) \
      .center(-10, 0) \
      .revolve()

#cone = cone.cut(cam).cut(bat)
cone = cone.cut(inner_cone) \
      .cut(bat) \
      .cut(cam) \
      .cut(rope_ring)


cone1 = cone.translate((0, 0, 0))
cone1 = cone1.cut(
          cq.Workplane("YZ") \
          .rect(2 * Rout + 50, 2 * h) \
          .extrude(-Rout - 50) \
          .rotate((0, 0, 0), (0, 0, 1), 30)
        ) \
        .translate((5, 0, 0))

#cone = cone.cut(
#          cq.Workplane("YZ") \
#          .rect(2 * Rout + 50, 2 * h) \
#          .extrude(Rout + 50) \
#          .rotate((0, 0, 0), (0, 0, 1), 30)
#        )

#      .line(Rout, 0) \

if __name__ == '__cqgi__':
    show_object(cone)
#    show_object(cone1)
#    show_object(x)
#    show_object(logo)
else:
    from cadquery import exporters

    box = cover.union(base)

    name = 'box_%dx%d' % (midi_sock_count, midi_sock_count)
    exporters.export(box, name + '.step')
    exporters.export(box, name + '.stl')
    exporters.export(box, name + '.vrml')
