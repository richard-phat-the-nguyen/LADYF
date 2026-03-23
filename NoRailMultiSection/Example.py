# Code qui génère une multisection de 4 profils sans rails
#Généré par l'IA copilot 2026-03-23


import adsk.core, adsk.fusion, traceback, random, math

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Use the active design (no new document)
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design found.")
            return

        root = design.rootComponent
        sketches = root.sketches
        planes = root.constructionPlanes

        basePlane = root.xYConstructionPlane

        # Z offsets for the 4 sketches
        z_offsets = ['0 cm', '2 cm', '4 cm', '6 cm']

        profiles = []

        for z in z_offsets:
            # Create offset plane
            planeInput = planes.createInput()
            planeInput.setByOffset(basePlane, adsk.core.ValueInput.createByString(z))
            offsetPlane = planes.add(planeInput)

            # Create sketch
            sketch = sketches.add(offsetPlane)

            # Generate random closed spline
            spline_points = adsk.core.ObjectCollection.create()

            num_points = random.randint(6, 8)
            base_radius = 1.0

            for i in range(num_points):
                angle = (2 * math.pi / num_points) * i
                r = base_radius + random.uniform(-0.3, 0.3)
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                spline_points.add(adsk.core.Point3D.create(x, y, 0))

            # Close the loop
            spline_points.add(spline_points.item(0))

            # Create spline curve
            sketch.sketchCurves.sketchFittedSplines.add(spline_points)

            # Grab the closed profile
            profiles.append(sketch.profiles.item(0))

        # Create loft
        lofts = root.features.loftFeatures
        loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        for p in profiles:
            loftInput.loftSections.add(p)

        loftInput.isSolid = True
        loftInput.isClosed = False

        lofts.add(loftInput)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
