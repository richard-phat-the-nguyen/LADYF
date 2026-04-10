# Code qui génère une multisection de 4 profils sans rails
# Généré par l'IA copilot 2026-03-23

import numpy as np
#import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import time

import adsk.core, adsk.fusion, adsk.cam, traceback

# ---------------------------------------------------------
# TES FONCTIONS — inchangées
# ---------------------------------------------------------

def bord_attaque(z):
    return 0.0152*z**3 - 0.1284*z**2 + 0.1048*z + 0.981

def bord_fuite(z):
    return 0.0063*z**3 - 0.0153*z**2 + 0.0133*z

def intersection(z):
    return bord_attaque(z) - bord_fuite(z)

def tubercle(z, corde=1, limite=1/3):
    seuil = fsolve(intersection, 0.5)[0] * limite
    A = 0.05 * corde
    Lambda = 0.5 * corde
    bord_attaque_tubercle = A * np.sin(2 * np.pi * (z - seuil) / Lambda)
    # Filtre
    Filtre = z < seuil
    bord_attaque_tubercle[Filtre] = 0
    return bord_attaque_tubercle

def organique(x, tnaca):
    y = 5.0 * tnaca * (
        0.2969 * np.sqrt(x)
        - 0.1260 * x
        - 0.3516 * x**2
        + 0.2843 * x**3
        - 0.1036 * x**4
    )
    return y

def programme_idrissa():
    z_npts = 150
    # Resolution de l'intersection
    z_inter = fsolve(intersection, 0.5)
    z = np.linspace(0, 0.995*z_inter[0], z_npts)
    #z = np.linspace(0, 4.24, z_npts)
    # Calcul du bord attaque et fuite avant modification des oscillation
    L_edge = bord_attaque(z)
    T_edge = bord_fuite(z)
    corde = L_edge - T_edge
    # IMPORTANT LA CORDE MOYENNE
    corde_moyenne = np.average(corde)
    oscillation = tubercle(z, corde_moyenne, 1/3)
    # Calcul des oscillations
    L_edge_corrige = L_edge + oscillation
    return [z, L_edge_corrige, T_edge]

# ---------------------------------------------------------
# FUSION 360 : création des 2 courbes dans le document actif
# ---------------------------------------------------------

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Aucun design actif. Ouvre un document Fusion 360 puis relance le script.")
            return

        root = design.rootComponent

        # Récupération des données depuis ton programme
        z, Le, Te = programme_idrissa()

        # Plan XZ
        xzPlane = root.xZConstructionPlane

        # Esquisse bord d'attaque
        skLE = root.sketches.add(xzPlane)
        skLE.name = "Leading Edge (tubercle)"

        # Esquisse bord de fuite
        skTE = root.sketches.add(xzPlane)
        skTE.name = "Trailing Edge"

        def add_spline(sketch, x_array, z_array):
            pts = adsk.core.ObjectCollection.create()
            for x_val, z_val in zip(x_array, z_array):
                # Courbe dans le plan XZ : (x, y=0, z)
                pts.add(adsk.core.Point3D.create(float(x_val), 0.0, float(z_val)))
            sketch.sketchCurves.sketchFittedSplines.add(pts)

        # Création des splines
        add_spline(skLE, Le, z)
        add_spline(skTE, Te, z)

        ui.messageBox("Courbes de bord d'attaque et de bord de fuite créées sur le plan XZ.")

    except:
        if ui:
            ui.messageBox("Échec :\n{}".format(traceback.format_exc()))

def stop(context):
    pass
