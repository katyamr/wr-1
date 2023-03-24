import cadquery as cq

w = 26.5
h = 17
l = 46
th = 1.8

holder = cq.Workplane("XY").rect(w, h) \
    .extrude(5) \
    .faces(">Z").shell(th) \
    .union(
        cq.Workplane("XY").rect(w, h) \
          .extrude(l) \
          .faces(">Y").shell(th)
    ) \
    .union(
        cq.Workplane("XY").rect(w, h) \
          .extrude(-4) \
          .faces("<Z").shell(th) \
          .translate((0, 0, l))
    ) \
    .cut(
        cq.Workplane("XY").rect(w - 2, h - 2) \
          .extrude(l) \
          .edges("|Z").fillet(7) \
          .translate((0, 0, -th))
    ) \
    .cut(
        cq.Workplane("XY").rect(w + th * 2, h + th) \
          .extrude(l - 8) \
          .edges("|X").fillet(4) \
          .translate((0, 2 + th, 4))
    ) \
    .cut(
        cq.Workplane("XY").rect(w - th * 2, h + th) \
          .extrude(l - 10) \
          .edges("|Y").fillet(4) \
          .translate((0, -th, 5))
    ) \
    .cut(
        cq.Workplane("XY").rect(2, h) \
          .extrude(10) \
          .translate((0, -th, -th))
    ) \
    .cut(
        cq.Workplane("XY").rect(2, h) \
          .extrude(10) \
          .translate((0, th, l - 10 + th))
    ) \
    .cut(
        cq.Workplane("XY").rect(2, h) \
          .extrude(10) \
          .translate((10, th, l - 10 + th))
    ) \
    .cut(
        cq.Workplane("XY").rect(2, h) \
          .extrude(10) \
          .translate((-10, th, l - 10 + th))
    ) \

top = holder \
    .translate((0, 0, 0)) \
    .faces(">Z").workplane(-l / 2 - th).split(keepTop=True) \
    .translate((0, - h - 5, -l )) \
    .rotate((0, 0, 0), (1, 0, 0), 180)

top = top.faces(">Y[-2]").workplane(centerOption="CenterOfBoundBox") \
    .center(0, -l / 4 + 2.5) \
    .rarray(6.5, 1, 3, 1, True) \
    .cboreHole(2, 4, th / 2)

holder = holder \
    .faces(">Z").workplane(-l / 2 - th).split(keepBottom=True)

holder = holder.faces("<Y[-2]").workplane(centerOption="CenterOfBoundBox") \
    .center(0, -l / 4 + 2.5) \
    .rarray(13, 1, 2, 1, True) \
    .cboreHole(2, 4, th / 2)

if __name__ == '__cqgi__':
    show_object(holder)
    show_object(top)
