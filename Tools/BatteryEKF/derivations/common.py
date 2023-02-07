from helpers import *
import sys

'''
predict:
SOC = SOC - dt*I/(Q*SOH)
SOH = SOH
R0 = R0
V1 = exp(-dt/(RC1))*V1+R1*I*(1-exp(-dt/(RC1)))
V2 = exp(-dt/(RC2))*V2+R2*I*(1-exp(-dt/(RC2)))
R1 = R1
R2 = R2

update:
V = SOC_to_OCV(SOC)-(I*R0)-V1-V2
'''

x = Matrix([])
state_idx_defines = []

def add_states(name, n):
    global x, state_idx_defines
    syms = []
    names = []

    if n == 1:
        names.append('%s' % (name.upper()))
        state_idx_defines.append('#define STATE_IDX_%s %u' % (names[-1], len(state_idx_defines)))
    else:
        for i in range(n):
            names.append('%s%u' % (name.upper(), i))
            state_idx_defines.append('#define STATE_IDX_%s %u' % (names[-1], len(state_idx_defines)))

    ret = Matrix([Symbol('x(STATE_IDX_%s)' % (name,), real=True) for name in names])
    x = Matrix([x, ret])
    return ret

def get_state_index(state_symbol):
    global x

    for i in range(len(x)):
        if x[i] == state_symbol:
            return i
    return None

# Battery EKF states
SOC = add_states('SOC', 1)[0]
SOH_inv = add_states('SOH_inv', 1)[0]
V1 = add_states('V1', 1)[0]
V2 = add_states('V2', 1)[0]
R0 = add_states('R0', 1)[0]
R1 = add_states('R1', 1)[0]
R2 = add_states('R2', 1)[0]

n_states = len(x)

P = copy_upper_to_lower_offdiagonals(Matrix(n_states, n_states, symbols(r'P(0:%u\,0:%u)' % (n_states,n_states), real=True)))

for i in range(n_states):
    P[i,i] = Symbol(str(P[i,i]), real=True, nonnegative=True)

V = Symbol('V', real=True)
V_sigma = Symbol('_params.V_sigma', real=True)
RC1  = Symbol('_params.RC1', real=True)
RC2  = Symbol('_params.RC2', real=True)
I = Symbol('I', real=True)
I_sigma = Symbol('_params.I_sigma',nonnegative=True)
Q = Symbol('_params.Q', real=True)
dt = Symbol('dt', positive=True)
temp_C = Symbol('temp_C', real=True)

OCV_from_SOC = Function("_model.OCV_from_SOC")
SOC_from_OCV = Function("_model.SOC_from_OCV")

#OCV_from_SOC = lambda SOC: 131.479653373276*SOC**7 - 491.355964059288*SOC**6 + 755.46607696667*SOC**5 - 619.46323086647*SOC**4 + 293.030671457998*SOC**3 - 80.0379786800851*SOC**2 + 12.5209316030758*SOC + 2.55535261270198

#SOC_from_OCV = lambda OCV: -8.42934372520677*OCV**8 + 225.183503842587*OCV**7 - 2619.24487212043*OCV**6 + 17324.6950169853*OCV**5 - 71269.0072323709*OCV**4 + 186710.134313024*OCV**3 - 304202.098847204*OCV**2 + 281817.793882536*OCV - 113661.252829786
