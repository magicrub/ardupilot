from math import sqrt, floor, fmod
from sympy import *
from sympy.printing.c import *
import math

class C99CodePrinterTweaked(C99CodePrinter):
    def __init__(self, *args, **kwargs):
        self.float_precision = kwargs.pop('fprec', 'double')
        assert self.float_precision in ['single', 'double']

        self.use_fast_trig = kwargs.pop('use_fast_trig', False)
        C99CodePrinter.__init__(self, *args, **kwargs)

        self.sqrt_fname = 'sqrt' if self.float_precision == 'double' else 'sqrtf'
        self.pow_fname = 'pow' if self.float_precision == 'double' else 'powf'

        if self.float_precision == 'single':
            self.known_functions = {
                "Abs": [(lambda x: not x.is_integer, "fabsf")],
                "gamma": "tgammaf",
                "sin": "sinf" if not self.use_fast_trig else "sinf_fast",
                "cos": "cosf" if not self.use_fast_trig else "cosf_fast",
                "tan": "tanf",
                "asin": "asinf",
                "acos": "acosf",
                "atan": "atanf",
                "atan2": "atan2f",
                "exp": "expf",
                "log": "logf",
                "erf": "erff",
                "sinh": "sinhf",
                "cosh": "coshf",
                "tanh": "tanhf",
                "asinh": "asinhf",
                "acosh": "acoshf",
                "atanh": "atanhf",
                "floor": "floorf",
                "ceiling": "ceilf",
            }

    def _print_Pow(self, expr):
        if "Pow" in self.known_functions:
            return self._print_Function(expr)
        PREC = precedence(expr)

        invert = expr.exp < 0

        if expr.exp == -1:
            ret = self._print(expr.base)
        else:
            if invert:
                expr = 1/(expr)

            if expr.exp == 0.5:
                ret = '%s(%s)' % (self.sqrt_fname, self._print(expr.base))
            elif expr.exp.is_integer and expr.exp <= 4:
                ret = "(%s)" % ('*'.join(["(%s)" % (self._print(expr.base)) for _ in range(expr.exp)]),)
            else:
                ret = '%s(%s, %s)' % (self.pow_fname, self._print(expr.base), self._print(expr.exp))

        if invert:
            return '1/(%s)' % (ret,)
        else:
            return ret

    def _print_Piecewise(self, expr):
        if expr.args[-1].cond != True:
            # We need the last conditional to be a True, otherwise the resulting
            # function may not return a result.
            raise ValueError("All Piecewise expressions must contain an "
                             "(expr, True) statement to be used as a default "
                             "condition. Without one, the generated "
                             "expression may not evaluate to anything under "
                             "some condition.")
        lines = []
        if expr.has(Assignment):
            for i, (e, c) in enumerate(expr.args):
                if i == 0:
                    lines.append("if (%s) {" % self._print(c))
                elif i == len(expr.args) - 1 and c == True:
                    lines.append("else {")
                else:
                    lines.append("else if (%s) {" % self._print(c))
                code0 = self._print(e)
                lines.append(code0)
                lines.append("}")
            return " ".join(lines)
        else:
            # The piecewise was used in an expression, need to do inline
            # operators. This has the downside that inline operators will
            # not work for statements that span multiple lines (Matrix or
            # Indexed expressions).
            ecpairs = ["((%s) ? ( %s ) " % (self._print(c),
                                               self._print(e))
                    for e, c in expr.args[:-1]]
            last_line = ": ( %s )" % self._print(expr.args[-1].expr)
            return ": ".join(ecpairs) + last_line + " ".join([")"*len(ecpairs)])

    def _print_Rational(self, expr):
        p, q = int(expr.p), int(expr.q)
        if self.float_precision == 'single':
            return '%d.0f/%d.0f' % (p, q)
        else:
            return '%d.0/%d.0' % (p, q)

def ccode_float(*args, **kwargs):
    return C99CodePrinterTweaked(fprec='single').doprint(*args, **kwargs)

def ccode_double(*args, **kwargs):
    return C99CodePrinterTweaked(fprec='double').doprint(*args, **kwargs)

def skew(_v):
    v = toVec(_v)
    assert v.rows == 3

    return Matrix([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def quat_from_euler(roll, pitch, yaw):
    return Matrix([
        sin(roll/2) * cos(pitch/2) * cos(yaw/2) + cos(roll/2) * sin(pitch/2) * sin(yaw/2),
        cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2),
        cos(roll/2) * cos(pitch/2) * sin(yaw/2) + sin(roll/2) * sin(pitch/2) * cos(yaw/2),
        cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        ])

def quat_to_euler(q):
    qi = q[0]
    qj = q[1]
    qk = q[2]
    qr = q[3]

    return Matrix([
        atan2(2*(qr*qi+qj*qk), 1-2*(qi**2+qj**2)),
        asin(2*(qr*qj-qk*qi)),
        atan2(2*(qr*qk+qi*qj), 1-2*(qj**2+qk**2))
        ])

def vec_norm(v):
    return sqrt(sum([x**2 for x in v]))

def toVec(*args):
    ret = Matrix(map(lambda x: Matrix([x]), args)).vec()
    return ret

def Rz(theta):
    return Matrix([[cos(theta), -sin(theta), 0],[sin(theta),cos(theta), 0], [0,0,1]])

def quat_to_gibbs(_q):
    q = toVec(_q)
    return q[0:3,:] / q[3]

def gibbs_to_quat(_g):
    g = toVec(_g)
    assert g.rows == 3
    return Matrix([
        [-g[0]/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)],
        [-g[1]/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)],
        [-g[2]/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)],
        [   -1/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)]
        ])

def gibbs_multiply(g1,g2):
    return g1+g2-g2.cross(g1) / (1-g1.dot(g2))

def rot_vec_to_quat_approx(_v):
    v = toVec(_v)
    assert v.rows == 3

    return toVec(v*Rational(1,2),1)

def quat_rotate(_q, _v):
    return quat_multiply(_q,rot_vec_to_quat(_v))

def quat_rotate_approx(_q, _v):
    return quat_multiply(_q,rot_vec_to_quat_approx(_v))

def rot_vec_to_quat(_v):
    v = toVec(_v)
    assert v.rows == 3

    theta = sqrt(v[0]**2+v[1]**2+v[2]**2)
    axis = v/Piecewise((theta, theta>0), (1, True))
    return toVec(sin(theta/2.) * axis[0], sin(theta/2.) * axis[1], sin(theta/2.) * axis[2], cos(theta/2.))

def quat_to_rot_vec_approx(_q):
    q = toVec(_q)
    return 2.*toVec(q[0],q[1],q[2])

def quat_to_rot_vec(_q):
    q = toVec(_q)
    assert q.rows == 4

    theta = 2*acos(q[3])
    l = sqrt(q[0]**2+q[1]**2+q[2]**2)

    axis = toVec(q[0],q[1],q[2])/Piecewise((l, l>0), (1, True))

    return theta*axis

def quat_inverse(_q):
    q = toVec(_q)
    assert q.rows == 4

    q[0] = -q[0]
    q[1] = -q[1]
    q[2] = -q[2]
    return q

def quat_normalize(_q):
    q = toVec(_q)
    assert q.rows == 4

    return q/sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2)

def quat_multiply(_q1, _q2):
    q1 = toVec(_q1)
    q2 = toVec(_q2)
    assert q1.rows == 4 and q2.rows == 4

    q1i = q1[0]
    q1j = q1[1]
    q1k = q1[2]
    q1w = q1[3]

    q2i = q2[0]
    q2j = q2[1]
    q2k = q2[2]
    q2w = q2[3]

    return toVec(q1w*q2i + q1i*q2w + q1j*q2k - q1k*q2j,
                 q1w*q2j - q1i*q2k + q1j*q2w + q1k*q2i,
                 q1w*q2k + q1i*q2j - q1j*q2i + q1k*q2w,
                 q1w*q2w - q1i*q2i - q1j*q2j - q1k*q2k)

def quat_to_matrix(_q):
    q = toVec(_q)
    assert q.rows == 4

    q_vec = q[:-1,0]
    q_w = q[3]
    return (q_w**2-(q_vec.T*q_vec)[0])*eye(3) + 2.*(q_vec*q_vec.T) + 2.*q_w*skew(q_vec)

def upperTriangularToVec(M):
    assert M.rows == M.cols

    N = M.rows
    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))
    return Matrix([M[r(k),c(k)] for k in range((N**2-N)/2+N)])

def uncompressSymMatrix(M):
    x = len(M)
    N = int(floor(sqrt(8*x + 1)/2 - 1/2))
    ret = zeros(N)
    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))
    for k in range(x):
        ret[r(k),c(k)] = ret[c(k),r(k)] = M[k]
    return ret


def copy_upper_to_lower_offdiagonals(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols

    ret = M[:,:]

    for r in range(ret.rows):
        for c in range(ret.cols):
            if r > c:
                ret[r,c] = ret[c,r]
    return ret

def compressedSymmetricMatrix(prefix, N):
    ret = zeros(N,N)

    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))

    for k in range((N**2-N)/2+N):
        ret[r(k),c(k)] = ret[c(k),r(k)] = Symbol('%s[%u]' % (prefix,k))
    return ret

def count_subexpression(subexpr, expr):
    if hasattr(expr, "__getitem__"):
        return sum(map(lambda x: count_subexpression(subexpr, x), expr))
    else:
        return expr.count(subexpr)

def extractSubexpressions(inexprs, prefix='X', threshold=0, prev_subx=[]):
    subexprs, outexprs = cse(inexprs, symbols=numbered_symbols('__TMP__'), order='none')

    subexprs = prev_subx+subexprs

    for i in reversed(range(len(subexprs))):
        from sympy.logic.boolalg import Boolean
        ops_saved = (count_subexpression(subexprs[i][0], [[x[1] for x in subexprs], outexprs])-1)*subexprs[i][1].count_ops()
        if ops_saved < threshold or isinstance(subexprs[i][1], Boolean):
            sub = dict([subexprs.pop(i)])
            subexprs = list(map(lambda x: (x[0],x[1].xreplace(sub)), subexprs))
            outexprs = list(map(lambda x: x.xreplace(sub), outexprs))

    for i in range(len(subexprs)):
        newSym = Symbol('%s%u' % (prefix,i+len(prev_subx)))
        sub = {subexprs[i][0]:newSym}
        subexprs[i] = (newSym,subexprs[i][1])
        subexprs = list(map(lambda x: (x[0],x[1].xreplace(sub)), subexprs))
        outexprs = list(map(lambda x: x.xreplace(sub), outexprs))

    outexprs = list(map(lambda x: Matrix(x) if type(x) is ImmutableDenseMatrix else x, outexprs))

    return tuple(outexprs+[subexprs])

def quickinv_sym(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols
    n = M.rows
    A = Matrix(n,n,symbols('_X[0:%u][0:%u]' % (n,n)))
    A = copy_upper_to_lower_offdiagonals(A)
    B = Matrix(simplify(A.inv()))
    return B.xreplace(dict(zip(A,M)))

def wrap_pi(x):
    while x >= math.pi:
        x -= 2*math.pi

    while x < -math.pi:
        x += 2*math.pi

    return x

def UDUdecomposition(M):
    assert M.rows == M.cols
    assert M.is_symmetric()

    P = M[:,:]

    n = P.rows

    U = zeros(*P.shape)
    D = zeros(*P.shape)

    for j in range(n-1, 0, -1):
        D[j,j] = P[j,j]
        alpha = 1/D[j,j]
        for k in range(j):
            beta = P[k,j]
            U[k,j] = alpha*beta
            for i in range(k+1):
                P[i,k] = P[i,k]-beta*U[i,j]
    D[0,0] = P[0,0]
    for i in range(n):
        U[i,i] = 1

    return U,D
