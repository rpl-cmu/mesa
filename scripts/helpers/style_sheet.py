import seaborn as sns

SNS_BLUE = "#0173b2"
SNS_ORANGE = "#de8f05"
SNS_GREEN = "#029e73"
SNS_RED = "#d55e00"
SNS_PURPLE = "#cc78bc"
SNS_BROWN = "#ca9161"
SNS_PINK = "#fbafe4"
SNS_GREY = "#949494"
SNS_YELLOW = "#ece133"
SNS_LIGHT_BLUE = "#56b4e9"

METHOD_STYLE_SHEET = {
    #################################
    # Constant Methods
    #################################
    "centralized": {
        "name": "Centralized",
        "color": SNS_GREY,
        "symbol": "*",
        "linestyle": "solid",
    },
    "independent": {
        "name": "Independent",
        "color": SNS_ORANGE,
        "symbol": "$I$",
        "linestyle": "solid",
    },
    "ddfsam": {
        "name": "DDF-SAM2",
        "color": SNS_LIGHT_BLUE,
        "symbol": "p",
        "linestyle": "solid",
    },
    #################################
    # MESA Methods
    #################################
    "geodesic-mesa": {
        "name": "MESA (Geodesic)",
        "color": SNS_BLUE,
        "symbol": "o",
        "linestyle": "solid",
    },
    "split-mesa": {
        "name": "MESA (Split)",
        "color": SNS_GREEN,
        "symbol": "s",
        "linestyle": "solid",
    },
    "chordal-mesa": {
        "name": "MESA (Chrodal)",
        "color": SNS_PINK,
        "symbol": "1",
        "linestyle": "solid",
    },
    "apxgeo-mesa": {
        "name": "MESA (Approx Geo)",
        "color": SNS_YELLOW,
        "symbol": "^",
        "linestyle": "solid",
    },
    #################################
    # Prior Works
    #################################
    "dgs": {
        "name": "DGS",
        "color": SNS_RED,
        "symbol": "P",
        "linestyle": "solid",
    },
    "mbadmm": {
        "name": "Multi-Block ADMM",
        "color": SNS_PURPLE,
        "symbol": "d",
        "linestyle": "solid",
    },
    "asapp": {
        "name": "ASAPP",
        "color": SNS_BROWN,
        "symbol": "X",
        "linestyle": "solid",
    },
}