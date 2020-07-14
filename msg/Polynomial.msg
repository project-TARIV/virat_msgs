# Communicate polynomials.

# Message should be in a static global frame
std_msgs/Header header

# This array consists of the coeffs of the polynomial:
# f(x) = a + b x + c x^2 + d x^3 ...
# where coeffs[0] = a; coeffs[1] = b; coeffs[2] = c ...
int8[] coeffs
