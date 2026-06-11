import adsk.core
import adsk.fusion
import math
import traceback


STUDENT = 'Александр Василенко — вариант 4'
VARIANT = 4
VARIANT_MATERIAL = 'PBT plastic'
VARIANT_LOAD_N = 400
BRACKET_X_POSITIONS = (-360, 0, 360)
WALL_ANCHOR_X_OFFSETS = (-35, 35)
WALL_ANCHOR_Z_POSITIONS = (-45, -145)
SHELF_SCREW_Y_POSITIONS = (75, 210)
TASK_6_GEAR_TEETH = (4, 8)
TASK_6_GEAR_MODULE = 10
TASK_6_CENTER_CLEARANCE = 10


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


def cross_product(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def dot_product(left, right):
    return sum(a * b for a, b in zip(left, right))


def basis_transform(origin, x_axis, y_axis):
    z_axis = cross_product(x_axis, y_axis)
    if (
        not math.isclose(dot_product(x_axis, x_axis), 1.0)
        or not math.isclose(dot_product(y_axis, y_axis), 1.0)
        or not math.isclose(dot_product(z_axis, z_axis), 1.0)
        or not math.isclose(dot_product(x_axis, y_axis), 0.0, abs_tol=1e-9)
    ):
        raise ValueError('Transform axes must be perpendicular unit vectors.')

    matrix = adsk.core.Matrix3D.create()
    succeeded = matrix.setWithCoordinateSystem(
        point(*origin),
        vector(*x_axis),
        vector(*y_axis),
        vector(*z_axis),
    )
    if not succeeded:
        raise RuntimeError('Fusion failed to create the component transform.')
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


def add_washer(root, name, outer_diameter, inner_diameter, depth, transform):
    component = new_component(root, name, transform)
    sketch = component.sketches.add(component.xYConstructionPlane)
    circles = sketch.sketchCurves.sketchCircles
    circles.addByCenterRadius(point(0, 0), mm(outer_diameter / 2))
    circles.addByCenterRadius(point(0, 0), mm(inner_diameter / 2))
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


def make_design(app):
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = design.rootComponent
    # Fusion owns the root component name and raises an error if it is changed.
    return design, root


def build_task_2(app):
    _, root = make_design(app)
    wall_holes = [
        (bracket_x + x_offset, z_position, 12)
        for bracket_x in BRACKET_X_POSITIONS
        for x_offset in WALL_ANCHOR_X_OFFSETS
        for z_position in WALL_ANCHOR_Z_POSITIONS
    ]
    shelf_holes = [
        (bracket_x, y_position - 150, 10)
        for bracket_x in BRACKET_X_POSITIONS
        for y_position in SHELF_SCREW_Y_POSITIONS
    ]

    add_box(
        root,
        'WALL — Steel — FIX REAR FACE',
        (1100, 500, 10),
        transform=basis_transform(
            (0, 0, 0),
            (1, 0, 0),
            (0, 0, 1),
        ),
        holes=wall_holes,
    )
    add_box(
        root,
        'SHELF — Steel — APPLY 400 N DOWN',
        (1000, 300, 3),
        origin=(0, 150, 0),
        holes=shelf_holes,
    )

    for index, x_pos in enumerate(BRACKET_X_POSITIONS, start=1):
        bracket = new_component(
            root,
            f'BRACKET {index} — ONE-PIECE RIBBED {VARIANT_MATERIAL}',
            identity_transform((x_pos, 0, 0)),
        )
        add_box(
            bracket,
            'HORIZONTAL SUPPORT PLATE — Bonded inside bracket',
            (130, 270, 10),
            origin=(0, 135, -10),
            holes=[
                (0, y_position - 135, 10)
                for y_position in SHELF_SCREW_Y_POSITIONS
            ],
        )
        add_box(
            bracket,
            'WALL MOUNTING PLATE — Bonded inside bracket',
            (130, 210, 10),
            transform=basis_transform(
                (0, 0, -95),
                (-1, 0, 0),
                (0, 0, 1),
            ),
            holes=[
                (x_offset, z_position + 95, 12)
                for x_offset in WALL_ANCHOR_X_OFFSETS
                for z_position in WALL_ANCHOR_Z_POSITIONS
            ],
        )
        for rib_index, rib_x in enumerate((-50, 42), start=1):
            add_triangle_prism(
                bracket,
                f'SIDE RIB {rib_index} — Bonded inside bracket',
                [(0, 0), (250, 0), (0, -170)],
                8,
                basis_transform(
                    (rib_x, 0, -10),
                    (0, 1, 0),
                    (0, 0, 1),
                ),
            )

        for x_offset in WALL_ANCHOR_X_OFFSETS:
            for z_position in WALL_ANCHOR_Z_POSITIONS:
                fastener_name = (
                    f'ANCHOR {index}.{x_offset:+d}.{abs(z_position)}'
                )
                anchor_x = x_pos + x_offset
                anchor_transform = basis_transform(
                    (anchor_x, -10, z_position),
                    (1, 0, 0),
                    (0, 0, -1),
                )
                add_cylinder(
                    root,
                    f'{fastener_name} SHANK — Steel — BONDED',
                    12,
                    22,
                    anchor_transform,
                )
                add_washer(
                    root,
                    f'{fastener_name} WASHER — EXCLUDE FROM SIMULATION',
                    28,
                    12,
                    2,
                    basis_transform(
                        (anchor_x, 10, z_position),
                        (1, 0, 0),
                        (0, 0, -1),
                    ),
                )
                add_cylinder(
                    root,
                    f'{fastener_name} HEAD — EXCLUDE FROM SIMULATION',
                    22,
                    8,
                    basis_transform(
                        (anchor_x, 12, z_position),
                        (1, 0, 0),
                        (0, 0, -1),
                    ),
                )

        for screw_index, y_position in enumerate(
            SHELF_SCREW_Y_POSITIONS,
            start=1,
        ):
            screw_name = f'SHELF SCREW {index}.{screw_index}'
            add_cylinder(
                root,
                f'{screw_name} SHANK — Steel — BONDED',
                10,
                15,
                identity_transform((x_pos, y_position, -10)),
            )
            add_washer(
                root,
                f'{screw_name} WASHER — EXCLUDE FROM SIMULATION',
                24,
                10,
                2,
                identity_transform((x_pos, y_position, 3)),
            )
            add_cylinder(
                root,
                f'{screw_name} HEAD — EXCLUDE FROM SIMULATION',
                18,
                6,
                identity_transform((x_pos, y_position, 5)),
            )

    add_label(root, (0, 340, 5))
    return (
        'Task 2 created.\n\n'
        'Assign PBT plastic to all BRACKET assemblies and Steel to wall, shelf, and fastener shanks.\n'
        'Exclude decorative fastener heads and washers from the simulation.\n'
        'Static Stress: fix rear wall face, apply 400 N downward to shelf, then set contacts per README.'
    )


def build_task_3(app):
    _, root = make_design(app)
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
    _, root = make_design(app)
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
    _, root = make_design(app)
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
    _, root = make_design(app)
    gear_1_teeth, gear_2_teeth = TASK_6_GEAR_TEETH
    module = TASK_6_GEAR_MODULE
    gear_1_radius = module * gear_1_teeth / 2
    gear_2_radius = module * gear_2_teeth / 2
    # Four-tooth simplified gears need extra center clearance to avoid overlap.
    gear_2_x = gear_1_radius + gear_2_radius + TASK_6_CENTER_CLEARANCE

    add_gear(
        root,
        'GEAR 1 — 4 teeth — DRIVER',
        gear_1_teeth,
        module,
        10,
        8,
        (0, 0, 0),
    )
    add_gear(
        root,
        'GEAR 2 — 8 teeth — DRIVEN',
        gear_2_teeth,
        module,
        10,
        10,
        (gear_2_x, 0, 0),
        22.5,
    )
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
        'Quick start: Ground AXLE 1, AXLE 2, and SLIDER GUIDE; add Revolute joints;\n'
        'add Motion Link 360 deg = -180 deg; then Drive Joint on GEAR 1.'
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
