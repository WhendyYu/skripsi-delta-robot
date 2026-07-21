import pandas as pd
import numpy as np

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression


# ============================================================
# LOAD DATASET
# ============================================================

# expected columns:
#
# x_true
# y_true
# x_measured
# y_measured

df = pd.read_excel(
    "calibration_result.xlsx"
)

# ============================================================
# EXTRACT DATA
# ============================================================

X_measured = df[[
    "x_measured",
    "y_measured"
]].values

Y_true = df[[
    "x_true",
    "y_true"
]].values


# ============================================================
# BUILD QUADRATIC FEATURES
# ============================================================

poly = PolynomialFeatures(

    degree=2,

    include_bias=True
)

X_poly = poly.fit_transform(
    X_measured
)


# ============================================================
# FIT X MODEL
# ============================================================

model_x = LinearRegression()

model_x.fit(

    X_poly,

    Y_true[:, 0]
)


# ============================================================
# FIT Y MODEL
# ============================================================

model_y = LinearRegression()

model_y.fit(

    X_poly,

    Y_true[:, 1]
)


# ============================================================
# FEATURE NAMES
# ============================================================

feature_names = poly.get_feature_names_out([
    "x",
    "y"
])


# ============================================================
# PRINT EQUATIONS
# ============================================================

# X EQUATION
coef_x = model_x.coef_
intercept_x = model_x.intercept_

print("f_x(x, y) = x_corrected")

print(
    f"= {intercept_x:.6f}"
    f" + ({coef_x[1]:.6f})x"
    f" + ({coef_x[2]:.6f})y"
    f" + ({coef_x[3]:.8f})x²"
    f" + ({coef_x[4]:.8f})xy"
    f" + ({coef_x[5]:.8f})y²"
)

print("\n")

# Y EQUATION
coef_y = model_y.coef_
intercept_y = model_y.intercept_

print("f_y(x, y) = y_corrected")

print(
    f"= {intercept_y:.6f}"
    f" + ({coef_y[1]:.6f})x"
    f" + ({coef_y[2]:.6f})y"
    f" + ({coef_y[3]:.8f})x²"
    f" + ({coef_y[4]:.8f})xy"
    f" + ({coef_y[5]:.8f})y²"
)