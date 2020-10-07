# Contains an array of points and other details

uint8 UNKNOWN=0

uint8 SOLID=1
uint8 DASHED=2
uint8 SOLID_DASHED=3
uint8 DASHED_SOLID=4
uint8 SOLID_SOLID=5
uint8 style

uint8 WHITE=1
uint8 YELLOW=2
uint8 color

uint8 OTHER=0
uint8 DRIVING_LEFT=1
uint8 DRIVING_RIGHT=2
uint8 line_pos

# Polynom coefficients polynom_coeffs[0], polynom_coeffs[1]*x, polynom_coeffs[2]*x^2, ...
float64[] polynom_coeffs
