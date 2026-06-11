import adsk.core
import adsk.fusion
import math
import traceback


STUDENT = 'Александр Василенко — вариант 4'
VARIANT = 4
VARIANT_MATERIAL = 'PBT plastic'
VARIANT_LOAD_N = 400


def mm(value):
    """Fusion API geometry uses centimeters internally."""
    return value / 10.0


def point(x, y, z=0):
    return adsk.core.Point3D.create(mm(x), mm(y), mm(z))


def vector(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def value_mm(value):
    return adsk.core.ValueInput.createByString(f'{value} mm')


def identity_transform(origin=(0, 0, 0)):
    matrix = adsk.core.Matrix3D.create()
    matrix.translation = vector(mm(origin[0]), mm(origin[1]), mm(origin[2]))
    return matrix


def basis_transform(origin, x_axis, y_axis, z_axis):
    matrix = adsk.core.Matrix3D.create()
    matrix.setWithCoordinateSystem(
        point(*origin),
        vector(*x_axis),
        vector(*y_axis),
        vector(*z_axis),
    )
    return matrix


def rotated_z_transform(origin, angle_deg):
    matrix = adsk.core.Matrix3D.create()
    matrix.setToRotation(
        math.radians(angle_deg),
        vector(0, 0, 1),
        point(0, 0, 0),
    )
    matrix.translation = vector(mm(origin[0]), mm(origin[1]), mm(origin[2]))
    return matrix


def new_component(root, name, transform=None):
    occurrence = root.occurrences.addNewComponent(
        transform if transform else adsk.core.Matrix3D.create()
    )
    occurrence.component.name = name
    return occurrence.component


def largest_profile(sketch):
    best = None
    best_area = -1
    for index in range(sketch.profiles.count):
        profile = sketch.profiles.item(index)
        area = profile.areaProperties().area
        if area > best_area:
            best = profile
            best_area = area
    if not best:
        raise RuntimeError('No closed profile was created.')
    return best


def extrude_profile(component, profile, depth_mm, body_name):
    extrudes = component.features.extrudeFeatures
    extrusion = extrudes.addSimple(
        profile,
        value_mm(depth_mm),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    body = extrusion.bodies.item(0)
    body.name = body_name
    return body


def add_box(root, name, size, origin=(0, 0, 0), transform=None, holes=None):
    component = new_component(
        root,
        name,
        transform if transform else identity_transform(origin),
    )
    sketch = component.sketches.add(component.xYConstructionPlane)
    curves = sketch.sketchCurves
    curves.sketchLines.addTwoPointRectangle(
        point(-size[0] / 2, -size[1] / 2),
        point(size[0] / 2, size[1] / 2),
    )
    for hole_x, hole_y, diameter in holes or []:
        curves.sketchCircles.addByCenterRadius(
            point(hole_x, hole_y),
            mm(diameter / 2),
        )
    extrude_profile(component, largest_profile(sketch), size[2], name)
    return component


def add_cylinder(root, name, diameter, depth, transform):
    component = new_component(root, name, transform)
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        point(0, 0),
        mm(diameter / 2),
    )
    extrude_profile(component, largest_profile(sketch), depth, name)
    return component


def add_triangle_prism(root, name, points_xy, depth, transform):
    component = new_component(root, name, transform)
    sketch = component.sketches.add(component.xYConstructionPlane)
    lines = sketch.sketchCurves.sketchLines
    vertices = [point(x, y) for x, y in points_xy]
    for index, start in enumerate(vertices):
        lines.addByTwoPoints(start, vertices[(index + 1) % len(vertices)])
    extrude_profile(component, largest_profile(sketch), depth, name)
    return component


def add_gear(
    root,
    name,
    teeth,
    module_mm,
    thickness_mm,
    bore_mm,
    origin,
    angle_deg=0,
):
    component = new_component(root, name, rotated_z_transform(origin, angle_deg))
    sketch = component.sketches.add(component.xYConstructionPlane)
    lines = sketch.sketchCurves.sketchLines

    pitch_radius = module_mm * teeth / 2
    outer_radius = pitch_radius + module_mm
    root_radius = max(bore_mm * 0.8, pitch_radius - 1.25 * module_mm)
    angular_pitch = 2 * math.pi / teeth
    outline = []

    for tooth in range(teeth):
        center_angle = tooth * angular_pitch
        samples = (
            (-0.50, root_radius),
            (-0.30, root_radius),
            (-0.19, outer_radius),
            (0.19, outer_radius),
            (0.30, root_radius),
        )
        for fraction, radius in samples:
            angle = center_angle + fraction * angular_pitch
            outline.append(point(radius * math.cos(angle), radius * math.sin(angle)))

    for index, start in enumerate(outline):
        lines.addByTwoPoints(start, outline[(index + 1) % len(outline)])

    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        point(0, 0),
        mm(bore_mm / 2),
    )
    extrude_profile(component, largest_profile(sketch), thickness_mm, name)
    return component


def add_label(root, origin, angle_deg=0):
    component = new_component(
        root,
        'LABEL — Александр Василенко — вариант 4',
        rotated_z_transform(origin, angle_deg),
    )
    sketch = component.sketches.add(component.xYConstructionPlane)
    try:
        text_input = sketch.sketchTexts.createInput2(STUDENT, mm(8))
        text_input.setAsMultiLine(
            point(-95, -8),
            point(95, 8),
            adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
            adsk.core.VerticalAlignments.MiddleVerticalAlignment,
            0,
        )
        sketch.sketchTexts.add(text_input)
    except Exception:
        # The component name remains visible in the browser on older Fusion builds.
        pass
    return component


def make_design(app, document_name):
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = design.rootComponent
    root.name = document_name
    return design, root


def build_task_2(app):
    _, root = make_design(app, 'Task 2 — Shelf static stress — Variant 4')
    bracket_x = (-400, -135, 135, 400)
    bolt_z = (-50, -130)
    wall_holes = [(x, z, 12) for x in bracket_x for z in bolt_z]

    add_box(
        root,
        'WALL — Steel — FIX REAR FACE',
        (1100, 500, 10),
        transform=basis_transform(
            (0, 0, 0),
            (1, 0, 0),
            (0, 0, 1),
            (0, -1, 0),
        ),
        holes=wall_holes,
    )
    add_box(
        root,
        'SHELF — Steel — APPLY 400 N DOWN',
        (1000, 300, 3),
        origin=(0, 150, 0),
    )

    for index, x_pos in enumerate(bracket_x, start=1):
        add_box(
            root,
            f'BRACKET {index} HORIZONTAL — {VARIANT_MATERIAL}',
            (80, 250, 8),
            origin=(x_pos, 125, -8),
        )
        add_box(
            root,
            f'BRACKET {index} VERTICAL — {VARIANT_MATERIAL}',
            (80, 180, 8),
            transform=basis_transform(
                (x_pos, 0, -90),
                (1, 0, 0),
                (0, 0, 1),
                (0, 1, 0),
            ),
            holes=[(0, 40, 12), (0, -40, 12)],
        )
        add_triangle_prism(
            root,
            f'BRACKET {index} RIB — {VARIANT_MATERIAL}',
            [(0, 0), (210, 0), (0, -150)],
            10,
            basis_transform(
                (x_pos - 5, 0, -8),
                (0, 1, 0),
                (0, 0, 1),
                (1, 0, 0),
            ),
        )

        for bolt_index, z_pos in enumerate(bolt_z, start=1):
            add_cylinder(
                root,
                f'BOLT {index}.{bolt_index} — Steel — BONDED',
                12,
                18,
                basis_transform(
                    (x_pos, -10, z_pos),
                    (1, 0, 0),
                    (0, 0, -1),
                    (0, 1, 0),
                ),
            )

    add_label(root, (0, 340, 5))
    return (
        'Task 2 created.\n\n'
        'Assign PBT plastic to all BRACKET components and Steel to wall, shelf, bolts.\n'
        'Static Stress: fix rear wall face, apply 400 N downward to shelf, then set contacts per README.'
    )


def build_task_3(app):
    _, root = make_design(app, 'Task 3 — Gear static stress — Variant 4')
    module = 10
    pitch_radius = module * VARIANT / 2
    outer_radius = pitch_radius + module
    root_radius = max(10 * 0.8, pitch_radius - 1.25 * module)
    center_distance = outer_radius + root_radius
    add_gear(root, 'GEAR A — 4 teeth — FIX BORE', 4, module, 12, 10, (0, 0, 0))
    add_gear(
        root,
        'GEAR B — 4 teeth — APPLY TORQUE',
        4,
        module,
        12,
        10,
        (center_distance, 0, 0),
        45,
    )
    add_label(root, (center_distance / 2, -75, 2))
    return (
        'Task 3 created.\n\n'
        'Assign Steel, create a Static Stress study, fix bore A, apply torque to bore B,\n'
        'and define Separation contact between the contacting tooth faces.'
    )


def build_task_4(app):
    _, root = make_design(app, 'Task 4 — Shape optimization — Variant 4')
    add_gear(
        root,
        'OPTIMIZATION CANDIDATE — 4 teeth',
        4,
        14,
        14,
        12,
        (0, 0, 0),
    )
    add_label(root, (0, -90, 2))
    return (
        'Task 4 created.\n\n'
        'Create Shape Optimization. Preserve the bore and tooth contact faces;\n'
        'apply a torque to the bore and tangential loads to the tooth faces.'
    )


def build_task_5(app):
    _, root = make_design(app, 'Task 5 — Dynamic impact — Variant 4')
    add_box(
        root,
        f'IMPACT PLATE — {VARIANT_MATERIAL} — 10 mm',
        (150, 150, 10),
        origin=(0, 0, 0),
    )
    add_box(
        root,
        'HAMMER HEAD — Steel — 20 x 20 contact face',
        (20, 20, 80),
        origin=(0, 0, 14),
    )
    add_box(
        root,
        'HAMMER HANDLE — Steel',
        (16, 120, 16),
        origin=(0, 65, 46),
    )
    add_label(root, (0, -110, 12))
    return (
        'Task 5 created.\n\n'
        'Assign PBT plastic to the plate and Steel to the hammer components.\n'
        'Dynamic Event Simulation: fix plate edges, use Separation contact, and give the hammer a downward initial velocity.'
    )


def build_task_6(app):
    _, root = make_design(app, 'Task 6 — Mechanism kinematics — Variant 4')
    module = 10
    gear_1_radius = module * 4 / 2
    gear_2_radius = module * 8 / 2
    gear_2_x = gear_1_radius + gear_2_radius

    add_gear(root, 'GEAR 1 — 4 teeth — DRIVER', 4, module, 10, 8, (0, 0, 0))
    add_gear(root, 'GEAR 2 — 8 teeth — DRIVEN', 8, module, 10, 10, (gear_2_x, 0, 0), 22.5)
    add_cylinder(root, 'AXLE 1 — Ground', 8, 18, identity_transform((0, 0, -4)))
    add_cylinder(root, 'AXLE 2 — Ground', 10, 18, identity_transform((gear_2_x, 0, -4)))
    add_cylinder(
        root,
        'CRANK PIN — Rigid Group with GEAR 2',
        8,
        26,
        identity_transform((gear_2_x + 25, 0, -4)),
    )
    add_box(
        root,
        'CONNECTING ROD — add Revolute Joints at ends',
        (140, 20, 8),
        origin=(gear_2_x + 87.5, 0, 10),
        holes=[(-62.5, 0, 8), (62.5, 0, 8)],
    )
    add_box(
        root,
        'SLIDER — add Slider Joint and Rigid Group with SLIDER PIN',
        (35, 35, 18),
        origin=(gear_2_x + 150, 0, 0),
        holes=[(0, 0, 8)],
    )
    add_cylinder(
        root,
        'SLIDER PIN — Rigid Group with SLIDER',
        8,
        26,
        identity_transform((gear_2_x + 150, 0, -4)),
    )
    add_box(
        root,
        'SLIDER GUIDE — Ground',
        (220, 50, 6),
        origin=(gear_2_x + 125, 0, -8),
    )
    add_label(root, (gear_2_x + 70, -90, 2))
    return (
        'Task 6 created.\n\n'
        'Ground both axles. Add Revolute joints to both gears and rod ends, a Slider joint to the slider,\n'
        'then Motion Link: 360 deg on gear 1 = -180 deg on gear 2.'
    )


BUILDERS = {
    '2': build_task_2,
    '3': build_task_3,
    '4': build_task_4,
    '5': build_task_5,
    '6': build_task_6,
}


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        choice, cancelled = ui.inputBox(
            'Введите номер задания: 2, 3, 4, 5 или 6',
            'ОМ ИД №3 — Александр Василенко — вариант 4',
            '2',
        )
        if cancelled:
            return
        choice = choice.strip()
        if choice not in BUILDERS:
            ui.messageBox('Нужно ввести одно число: 2, 3, 4, 5 или 6.')
            return

        message = BUILDERS[choice](app)
        app.activeViewport.fit()
        ui.messageBox(message)
    except Exception:
        if ui:
            ui.messageBox('Ошибка генератора:\n' + traceback.format_exc())


def stop(context):
    pass
