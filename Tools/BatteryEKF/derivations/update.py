from common import *

z = Matrix([V])
R = Matrix([((I_sigma)*R0 + V_sigma)**2])
# Tag obs model
h = Matrix([OCV_from_SOC(SOC, temp_C)-R0*I-V1-V2])
y = z-h                       # Innovation
H = h.jacobian(x)   # Obervation sensitivity matrix
S = H*P*H.T + R                      # Innovation covariance
S_I = quickinv_sym(S)                # Innovation covariance inverse
K = P*H.T*S_I                        # Near-optimal Kalman gain
NIS = y.T*S_I*y                      # Normalized innovation squared
x_n = x+K*y                # Updated state vector

P_n = (eye(len(x),len(x))-K*H)*P*(eye(len(x),len(x))-K*H).T+K*R*K.T    # Updated covariance matrix

subs = {OCV_from_SOC(SOC, temp_C).fdiff(): Function("_model.OCV_from_SOC_diff")(SOC, temp_C)}

y = y.subs(subs)
NIS = NIS.subs(subs)
x_n = x_n.subs(subs)
P_n = P_n.subs(subs)

y, NIS, x_n, P_n, subx = extractSubexpressions([y,NIS,x_n,P_n],'subx',threshold=4)

with open(sys.argv[1], 'w') as f:
    f.write('    { //////// Begin generated code: Update model      ////////\n')
    for i in range(len(subx)):
        f.write('        double %s = %s;\n' % (subx[i][0], ccode_double(subx[i][1])))

    f.write('')

    f.write('        y = %s;\n' % (ccode_double(y[0]),))

    f.write('        NIS = %s;\n' % (ccode_double(NIS[0]),))
    f.write('')

    for i in range(len(x_n)):
        f.write('        x_n(%u,0) = %s;\n' % (i, ccode_double(x_n[i])))

    f.write('')

    for i in range(P_n.rows):
        for j in range(P_n.cols):
            #if P_n[i,j] != 0:
                f.write('        P_n(%u,%u) = %s;\n' % (i, j, ccode_double(P_n[i,j])))
    f.write('    } //////// End generated code: Update model        ////////\n')
