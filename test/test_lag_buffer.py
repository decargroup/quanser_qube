"""Simple script to generate test vectors for ``test_lag_buffer.cpp``."""

import numpy as np
import control

def c_print(y, name):
    print(f'float {name}[{len(y)}] = ', end='')
    print('{', end='')
    for k in range(len(y) - 1):
        print(y[k], end=', ')
    print(y[-1], end='')
    print('};')

sys = control.TransferFunction([6, 4, 2], [1, 3, 1], True)
inpt1 = np.arange(0, 1, 0.1)
_, y1 = control.forced_response(sys, T=np.arange(len(inpt1)), U=inpt1)

print('LTI system responds correctly compared to Python-generated test vectors')
c_print(inpt1, 'input')
c_print(y1, 'output')