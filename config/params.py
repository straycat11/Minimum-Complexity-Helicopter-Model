params = {
    "FS.CG": 132.7,
    "WL.CG": 38.5,
    "WT": 5401.0,
    "Ix": 1300.0,
    "Iy": 6760.0,
    "Iz": 6407.0,
    "Ixz": 800.0,
    "dt" : 0.01, # step size
    "a2" : 0.5, # numerical integration constant
    "interactions_mr" : False, # turn main rotor interactions on
    "interactions_tr" : False, # turn tail rotor interactions on
    "rho_sea_level": 0.0023769,
    "main_rotor": {
        "FS.HUB": 132.4, # FT (fuselage station)
        "WL.HUB": 98.2, # FT (waterline)
        "IS": .11, # Main rotor shaft incidence (rad)
        "E.MR": 0.50,
        "IB": 212.0,
        "R.MR": 18.0,
        "A.MR": 6.0,
        "RPM.MR": 385.0,
        "CD0": 0.010,
        "B.MR": 4.0,
        "C.MR": 1.10,
        "TWST.MR": -0.105,
        "K1": 0.0,
    },
    "fuselage": {
        "FS.FUS": 132.0, # FT
        "WL.FUS": 38.0, # FT
        "XUU.FUS": -10.8,
        "YVV.FUS": -167.0,
        "ZWW.FUS": -85.0,
    },
    "wing": {
        "FS.WN": 0.0,
        "WL.WN": 0.0,
        "ZUU.WN":0.0,
        "ZUW.WN": 0.0,
        "ZMAX.WN": 0.0,
        "B.WN": 1.0,
    },
    "horizontal_tail": {
        "FS.HT": 330.0,
        "WL.HT": 54.0,
        "ZUU.HT":.4,
        "ZUW.HT": -34.0,
        "ZMAX.HT": -22.0,
    },
    "vertical_tail": {
        "FS.VT": 380.0,
        "WL.VT": 80.0,
        "YUU.VT": 3.3,
        "YUV.VT": -47.0,
        "YMAX.VT": -17.0,
    },
    "tail_rotor": {
        "FS.TR": 391.0,
        "WL.TR": 70.0,
        "R.TR": 3.1,
        "A.TR": 3.0,
        "SOL.TR": .134,
        "RPM.TR": 2080.0,
        "TWST.TR": -.137,
    },
}
