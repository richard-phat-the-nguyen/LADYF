#Code inspiré d'IA copilot et modifié par la suite

import adsk.core, adsk.fusion, traceback
import numpy as np
import math

# ---------- NACA-like geometry functions (from your script) ----------
def bord_attaque(x):
    return 0.0152 * x**3 - 0.1284 * x**2 + 0.1048 * x + 0.981

def bord_fuite(x):
    return 0.0063 * x**3 - 0.0153 * x**2 + 0.0133 * x

def intersection(x):
    return bord_attaque(x) - bord_fuite(x)

def organique(x, tnaca):
    y = 5.0 * tnaca * (
        0.2969 * np.sqrt(x)
        - 0.1260 * x
        - 0.3516 * x**2
        + 0.2843 * x**3
        - 0.1036 * x**4
    )
    return y

# ---------- Main Fusion 360 script ----------
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Use the active design (do not create a new document)
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion design found.')
            return

        root = design.rootComponent
        sketches = root.sketches
        planes = root.constructionPlanes
        basePlane = root.xYConstructionPlane

        # Recreate the same x,y profile sampling as in your plotting script
        beta = np.linspace(0, np.pi, 100)
        x = (1 - np.cos(beta)) / 2.0
        y = organique(x, 0.21)

        # Solve intersection to get the z limit (use a small margin)
        # If scipy is not available in Fusion's Python, replace with a numeric root finder or a fixed z limit.
        try:
            from scipy.optimize import fsolve
            z_inter = fsolve(intersection, 0.5)
            z_inter_val = 0.99 * float(z_inter[0])
        except Exception:
            # fallback: use a conservative z limit if scipy isn't available
            z_inter_val = 0.5

        z2_npts = 100
        z2 = np.linspace(0.0, z_inter_val, z2_npts)

        # Collect profiles for loft
        profiles = []

        for i in range(z2_npts):
            z_val = float(z2[i])

            # compute chord and scaled profile at this z
            corde = bord_attaque(z_val) - bord_fuite(z_val)
            x2 = -x * corde                      # negative as in your plotting code
            y_upper = y * corde
            y_lower = -y_upper

            # translate in X by leading edge position (bord_attaque)
            x_offset = bord_attaque(z_val)
            x2_translated = x2 + x_offset

            # Create an offset construction plane parallel to XY at z = z_val
            planeInput = planes.createInput()
            # Use a string with units so Fusion interprets it (cm). Change unit if needed.
            planeInput.setByOffset(basePlane, adsk.core.ValueInput.createByString(f"{z_val:.6f} cm"))
            offsetPlane = planes.add(planeInput)

            # Create sketch on that plane
            sketch = sketches.add(offsetPlane)

            # Create two point collections: upper and lower
            upper_pts = adsk.core.ObjectCollection.create()
            lower_pts = adsk.core.ObjectCollection.create()

            # Add upper surface points in order (leading -> trailing)
            for j in range(len(x2_translated)):
                px = float(x2_translated[j])
                py = float(y_upper[j])
                upper_pts.add(adsk.core.Point3D.create(px, py, 0.0))

            # Add lower surface points in order (trailing -> leading)
            # We'll add them reversed so that when combined they form a closed loop if needed.
            for j in range(len(x2_translated)-1, -1, -1):
                px = float(x2_translated[j])
                py = float(y_lower[j])
                lower_pts.add(adsk.core.Point3D.create(px, py, 0.0))

            # Create fitted splines for upper and lower curves
            sketchCurves = sketch.sketchCurves
            splines = sketchCurves.sketchFittedSplines
            upper_spline = splines.add(upper_pts)
            lower_spline = splines.add(lower_pts)

            # Ensure the trailing edge is closed: draw a short line between the trailing points
            # trailing point is the last point of upper_pts and first point of lower_pts (same x)
            trailing_upper = upper_pts.item(upper_pts.count - 1)
            trailing_lower = lower_pts.item(0)
            # If they are not identical (numerical), create a closing line
            if (abs(trailing_upper.x - trailing_lower.x) > 1e-9) or (abs(trailing_upper.y - trailing_lower.y) > 1e-9):
                sketchCurves.sketchLines.addByTwoPoints(trailing_upper, trailing_lower)

            # Also ensure leading edge is closed: connect last lower point to first upper point if needed
            leading_upper = upper_pts.item(0)
            leading_lower = lower_pts.item(lower_pts.count - 1)
            if (abs(leading_upper.x - leading_lower.x) > 1e-9) or (abs(leading_upper.y - leading_lower.y) > 1e-9):
                sketchCurves.sketchLines.addByTwoPoints(leading_lower, leading_upper)

            # At this point the sketch should contain a closed loop. Grab the first profile.
            # If multiple profiles exist, try to pick the one with the largest area (most likely the airfoil).
            if sketch.profiles.count == 0:
                ui.messageBox(f"No profile found in sketch at z={z_val:.6f} cm. Sketch profiles: 0")
                continue

            # If only one profile, use it. If multiple, pick the largest area profile.
            chosen_profile = None
            if sketch.profiles.count == 1:
                chosen_profile = sketch.profiles.item(0)
            else:
                # pick profile with max area
                max_area = -1.0
                for p_idx in range(sketch.profiles.count):
                    p = sketch.profiles.item(p_idx)
                    # compute area via profile.areaProperties if available
                    try:
                        props = p.areaProperties
                        area = abs(props.area)
                    except:
                        # fallback: use bounding box area (less accurate)
                        bbox = p.boundingBox
                        area = abs((bbox.maxPoint.x - bbox.minPoint.x) * (bbox.maxPoint.y - bbox.minPoint.y))
                    if area > max_area:
                        max_area = area
                        chosen_profile = p

            if chosen_profile:
                profiles.append(chosen_profile)
            else:
                ui.messageBox(f"Could not determine a profile at z={z_val:.6f} cm.")

        # Create loft through the collected profiles (no rails)
        if len(profiles) >= 2:
            loftFeats = root.features.loftFeatures
            loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

            for prof in profiles:
                loftInput.loftSections.add(prof)

            loftInput.isSolid = True
            loftInput.isClosed = False
            loftFeats.add(loftInput)
        else:
            ui.messageBox('Not enough profiles to create a loft.')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
