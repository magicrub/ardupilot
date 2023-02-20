from common import *

# Generate prediction model

# f: state-transtition model for the purpose of linearization
f = Matrix([SOC-dt*I*SOH_inv/Q,
            SOH_inv,
            exp(-dt/(RC1))*V1 + R1*I*(1-exp(-dt/(RC1))),
            exp(-dt/(RC2))*V2+R2*I*(1-exp(-dt/(RC2))),
            R0,
            R1,
            R2])

F = f.jacobian(x)

# u: control input vector
u = Matrix([I])

# G: control-influence matrix, AKA "B" in literature
G = f.jacobian(u)

# w_u_sigma: additive noise on u
w_u_sigma = Matrix([I_sigma])

# Q_u: covariance of additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

# _Q: covariance of additive noise on x
_Q = G*Q_u*G.T

P_n = F*P*F.T + _Q

pprint(_Q)

#P_n[0,0] += (sqrt(P[0,0])+dt*I_sigma*SOH_inv/Q)**2-P[0,0]

# Generate C code for prediction model
x_n, P_n, subx = extractSubexpressions([f, P_n], 'subx', threshold=4)

with open(sys.argv[1], 'w') as f:
    f.write('    { //////// Begin generated code: Prediction model      ////////\n')
    for i in range(len(subx)):
        f.write('        double %s = %s;\n' % (subx[i][0], ccode_double(subx[i][1])))

    f.write('\n')

    for i in range(len(x_n)):
        f.write('        x_n(%u,0) = %s;\n' % (i, ccode_double(x_n[i])))

    f.write('\n')

    for i in range(P_n.rows):
        for j in range(P_n.cols):
            #if P_n[i,j] != 0:
                f.write('        P_n(%u,%u) = %s;\n' % (i, j, ccode_double(P_n[i,j])))
    f.write('    } //////// End generated code: Prediction model        ////////\n')

# Generate C code for prediction model
#Q, subx = extractSubexpressions([Q], 'subx', threshold=4)

#print('{ //////// Begin generated code: Process noise         ////////')
#for i in range(len(subx)):
    #print('    float %s = %s;' % (subx[i][0], ccode_float(subx[i][1])))

#print ('')

#for i in range(Q.rows):
    #for j in range(Q.cols):
        #if Q[i,j] != 0:
            #print('    P(%u,%u) += %s;' % (i, j, ccode_float(Q[i,j])))
#print('} //////// End generated code: Process noise           ////////\n')


