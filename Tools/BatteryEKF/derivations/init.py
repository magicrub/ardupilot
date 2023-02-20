from common import *

# Generate covariance initialization
SOH_init = Symbol('_params.SOH_init', real=True)
R0_init  = Symbol('_params.R0_init', real=True)
R1_init  = Symbol('_params.R1_init', real=True)
R2_init  = Symbol('_params.R2_init', real=True)
I_step   = Symbol('_params.I_step', real=True)

SOH_sigma    = Symbol('_params.SOH_sigma', real=True)
R0_sigma     = Symbol('_params.R0_sigma', real=True)
R1_sigma     = Symbol('_params.R1_sigma', real=True)
R2_sigma     = Symbol('_params.R2_sigma', real=True)
I_step_sigma = Symbol('_params.I_step_sigma', real=True)

V1_init = (I+I_step)*R1_init
V2_init = (I+I_step)*R2_init

initial_state = Matrix([
    SOC_from_OCV(V+I*R0_init+V1_init+V2_init, temp_C),
    1/SOH_init,
    V1_init,
    V2_init,
    R0_init,
    R1_init,
    R2_init])

init_variables = Matrix([V, I, SOH_init, R0_init, R1_init, R2_init, I_step])

init_variable_influence = initial_state.jacobian(init_variables)

init_variables_sigma = Matrix([V_sigma, I_sigma, SOH_sigma, R0_sigma, R1_sigma, R2_sigma, I_step_sigma])

init_variables_cov = diag(*init_variables_sigma.multiply_elementwise(init_variables_sigma))

initial_state_cov = init_variable_influence*init_variables_cov*init_variable_influence.T

subs = {I_step:0, SOC_from_OCV(V+I*R0_init+V1_init+V2_init, temp_C).fdiff(): Function("_model.SOC_from_OCV_diff")(V+I*R0_init+V1_init+V2_init, temp_C), R0_init:0, R1_init:0, R2_init:0, I: 1.}

initial_state = initial_state.subs(subs)
initial_state_cov = initial_state_cov.subs(subs)

pprint(initial_state_cov)

initial_state, initial_state_cov, subx = extractSubexpressions([initial_state,initial_state_cov], 'subx', threshold=4)


# Generate C code for quaternion covariance initialization
with open(sys.argv[1], 'w') as f:
    f.write('    { //////// Begin generated code: Initial state covariance ////////\n')
    for i in range(len(subx)):
        f.write('        float %s = %s;\n' % (subx[i][0], ccode_double(subx[i][1])))

    f.write('\n')

    for i in range(initial_state.rows):
        f.write('        initial_state(%u) = %s;\n' % (i, ccode_double(initial_state[i])))

    f.write('\n')

    for i in range(initial_state_cov.rows):
        for j in range(initial_state_cov.cols):
            if initial_state_cov[i,j] != 0:
                f.write('        initial_state_cov(%u,%u) = %s;\n' % (i, j, ccode_double(initial_state_cov[i,j])))

    f.write('    } //////// End generated code: Initial state covariance   ////////\n')
