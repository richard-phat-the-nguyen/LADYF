#Code inspiré par les IA ChatGPT et Claude et modifié pour implémenté le tracage des nageoirs des baleines


import adsk.core
import adsk.fusion
import traceback
import numpy as np
from scipy.optimize import fsolve


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def bord_attaque(x):
    return 0.0152*x**3 - 0.1284*x**2 + 0.1048*x + 0.981

def bord_fuite(x):
    return 0.0063*x**3 - 0.0153*x**2 + 0.0133*x

def intersection(x):
    return bord_attaque(x) - bord_fuite(x)

def Naca_4_digits(Naca="0012", npts=100, echelle=1, centered=False):
    M  = int(Naca[0]) / 100
    P  = int(Naca[1]) / 10
    XX = int(Naca[2:4]) / 100

    beta = np.linspace(0, np.pi, npts)
    x    = (1 - np.cos(beta)) / 2

    yc    = np.array(x)
    dy_dx = np.zeros(npts)

    for ii in range(len(x)):
        if x[ii] >= 0 and x[ii] < P:
            yc[ii]    = M / P**2 * (2*P*x[ii] - x[ii]**2)
            dy_dx[ii] = 2*M / P**2 * (P - x[ii])
        elif x[ii] >= P and x[ii] <= 1:
            yc[ii]    = M / (1-P)**2 * (1 - 2*P + 2*P*x[ii] - x[ii]**2)
            dy_dx[ii] = 2*M / (1-P)**2 * (P - x[ii])

    t  = np.arctan(dy_dx)
    a0, a1, a2, a3, a4 = 0.2969, -0.126, -0.3516, 0.2843, -0.1036
    T  = XX
    yt = T / 0.2 * (a0*x**0.5 + a1*x + a2*x**2 + a3*x**3 + a4*x**4)

    yu = yc + yt * np.cos(t)
    yl = yc - yt * np.cos(t)

    xf = np.hstack((x,    x[::-1]))
    yf = np.hstack((yu, yl[::-1]))

    if centered:
        xf = xf - 0.3394

    return [xf * echelle, yf * echelle]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run(context):
    ui = None
    try:
        app    = adsk.core.Application.get()
        ui     = app.userInterface
        design = app.activeProduct

        if not isinstance(design, adsk.fusion.Design):
            ui.messageBox('No active Fusion design.', 'Error')
            return

        rootComp = design.rootComponent

        # ------------------------------------------------------------------
        # Span parameter
        # ------------------------------------------------------------------
        y_inter    = fsolve(intersection, 0.5) * 0.995
        y_vals     = np.linspace(0, y_inter[0], 100)
        span_end   = float(y_vals[-1])          # Y coordinate of fin tip

        # ------------------------------------------------------------------
        # 1️⃣  LEADING EDGE RAIL  — on XY plane, spans Y = 0 → span_end
        # ------------------------------------------------------------------
        x_le = bord_attaque(y_vals)

        sketch_le = rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch_le.is3D= True
        # ICI ajout
        pts_le    = adsk.core.ObjectCollection.create()
        for i in range(len(y_vals)):
            pts_le.add(adsk.core.Point3D.create(x_le[i], y_vals[i], 0.0))
        spline_le = sketch_le.sketchCurves.sketchFittedSplines.add(pts_le)

        bord_attaque_0          = float(x_le[0])   # LE chord at root  (Y=0)
        bord_attaque_av_dernier = float(x_le[-1])  # LE chord at tip   (Y=span_end)

        # ------------------------------------------------------------------
        # 2️⃣  TRAILING EDGE RAIL — on XY plane, same Y range
        # ------------------------------------------------------------------
        x_te = bord_fuite(y_vals)

        # sketch_te = rootComp.sketches.add(rootComp.xYConstructionPlane)
        # sketch_te.is3D=True
        pts_te    = adsk.core.ObjectCollection.create()
        for i in range(len(y_vals)):
            pts_te.add(adsk.core.Point3D.create(x_te[i], y_vals[i], 0.0))
        spline_te=sketch_le.sketchCurves.sketchFittedSplines.add(pts_te)

        bord_fuite_0          = float(x_te[0])    # TE chord at root
        bord_fuite_av_dernier = float(x_te[-1])   # TE chord at tip

        # ------------------------------------------------------------------
        # Rail endpoint summary (world coords, for reference):
        #   LE root : (bord_attaque_0,          0,         0)
        #   LE tip  : (bord_attaque_av_dernier,  span_end,  0)
        #   TE root : (bord_fuite_0,             0,         0)
        #   TE tip  : (bord_fuite_av_dernier,    span_end,  0)
        #
        # ➜ Profiles must be flat in Z=0 and positioned at Y=0 (root)
        #   and Y=span_end (tip) so their LE/TE points coincide with
        #   the rail endpoints above.
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # 3️⃣  ROOT PROFILE — XZ plane (world Y = 0), thickness along Z
        # ------------------------------------------------------------------
        sketch_root = rootComp.sketches.add(rootComp.xZConstructionPlane)

        L_root = bord_attaque_0 - bord_fuite_0
        xn, yn = Naca_4_digits("2412", 100, 1, False)
        # Map normalised NACA [0,1] onto chord, flip so LE → bord_attaque_0
        xr = -xn * L_root + bord_attaque_0   # X in [bord_fuite_0, bord_attaque_0]
        zr = -yn * L_root                    # thickness in Z

        pts_root = adsk.core.ObjectCollection.create()
        for i in range(len(xr)):
            # World coords on xZConstructionPlane: (X, Y=0, Z)
            pts_root.add(adsk.core.Point3D.create(xr[i], zr[i],0 ))
        sketch_root.sketchCurves.sketchFittedSplines.add(pts_root)

        # ------------------------------------------------------------------
        # 4️⃣  TIP PROFILE — offset plane at world Y = span_end
        # ------------------------------------------------------------------
        # setByOffset on xZConstructionPlane offsets along its normal = world Y
        plane_input = rootComp.constructionPlanes.createInput()
        plane_input.setByOffset(
            rootComp.xZConstructionPlane,
            adsk.core.ValueInput.createByReal(span_end)
        )
        tip_plane  = rootComp.constructionPlanes.add(plane_input)
        sketch_tip = rootComp.sketches.add(tip_plane)

        L_tip = bord_attaque_av_dernier - bord_fuite_av_dernier
        xn, yn = Naca_4_digits("2412", 100, 1, False)
        xt = -xn * L_tip + bord_attaque_av_dernier
        zt = -yn * L_tip

        pts_tip = adsk.core.ObjectCollection.create()
        for i in range(len(xt)):
            # Offset plane local axes: u→X, v→Z (world Y handled by offset)
            # Pass world coords directly; Fusion projects onto the sketch plane
            # pts_tip.add(adsk.core.Point3D.create(xt[i], span_end, zt[i]))
            pts_tip.add(adsk.core.Point3D.create(xt[i], zt[i],0 ))
        sketch_tip.sketchCurves.sketchFittedSplines.add(pts_tip)

        # ------------------------------------------------------------------
        # 5️⃣  LOFT
        # ------------------------------------------------------------------
        # Sanity-check: profiles must have at least one closed profile
        # if sketch_root.profiles.count == 0:
        #     ui.messageBox('Root profile is empty — check coordinate mapping.')
        #     return
        # if sketch_tip.profiles.count == 0:
        #     ui.messageBox('Tip profile is empty — check coordinate mapping.')
        #     return

        # loftFeats = rootComp.features.loftFeatures
        # loftInput = loftFeats.createInput(
        #     adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        # )

        # # Sections: root first, tip second
        # loftInput.loftSections.add(sketch_root.profiles.item(0))
        # loftInput.loftSections.add(sketch_tip.profiles.item(0))

        # # Guide rails: LE and TE splines
        # loftInput.centerLineOrRails.addRail(spline_le)
        # loftInput.centerLineOrRails.addRail(spline_te)

        # loftInput.isSolid  = True
        # loftInput.isClosed = False

        # loftFeats.add(loftInput)

        # ui.messageBox('Whale fin loft created successfully!')
        
        lofts = root.features.loftFeatures
        loftInput = lofts.createInput(
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        )

        # Add loft sections
        loftInput.loftSections.add(sketch_root.profiles.item(0))
        loftInput.loftSections.add(sketch_tip.profiles.item(0))

        # Add both rails
        loftInput.centerLineOrRails.addRail(spline_le)
        loftInput.centerLineOrRails.addRail(spline_te)

        # Create loft feature
        loft = lofts.add(loftInput)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
