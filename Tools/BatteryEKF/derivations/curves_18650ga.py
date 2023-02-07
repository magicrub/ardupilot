import numpy as np
import sympy as sp
import sympy.utilities.lambdify as lambdify
from scipy import interpolate

#def polyfit(data, deg=8, inverse=False):
    #if inverse:
        #return np.polyfit(data['y'], data['x'], deg)
    #else:
        #return np.polyfit(data['x'], data['y'], deg)

#def polynomial_expression(coeffs, xname, yname):
    #ret = yname+' = '
    #ret += ' + '.join(["%.15f*%s**%d" % (coeffs[i],xname,len(coeffs)-i-1) for i in range(len(coeffs)-2)])
    #ret += " + %.15f*%s" % (coeffs[-2],xname)
    #ret += " + %.15f" % (coeffs[-1],)
    #return ret

class VCurve:
    def __init__(self, capacity, current, voltdata, tempdata, deg=8):
        self.SOC_sym = sp.Symbol('SOC', real=True)
        self.V_sym = sp.Symbol('V', real=True)

        self.current=current

        x = 1-np.asarray(voltdata['x'])/capacity
        y = np.asarray(voltdata['y'])
        self.v_points_x = x
        self.v_points_y = y

        interp_x_points = np.linspace(-0.01, 1.01, 10000)

        interp_y_func = interpolate.interp1d(list(reversed(x)), list(reversed(y)), fill_value='extrapolate')
        interp_y_points = interp_y_func(interp_x_points)

        self.v_coeffs = np.polyfit(interp_x_points, interp_y_points, deg)
        self.inv_v_coeffs = np.polyfit(interp_y_points,interp_x_points,deg)
        self.v_func = np.poly1d(self.v_coeffs)
        self.inv_v_func = np.poly1d(self.inv_v_coeffs)
        self.v_from_soc = sp.lambdify([self.SOC_sym],self._get_sympy_expr(self.SOC_sym, self.v_coeffs))
        self.soc_from_v = sp.lambdify([self.V_sym],self._get_sympy_expr(self.V_sym, self.inv_v_coeffs))
        x = 1-np.asarray(tempdata['x'])/capacity
        y = np.asarray(tempdata['y'])

        interp_x_points = np.linspace(x[0], x[-1], 10000)

        interp_y_points = np.interp(interp_x_points, list(reversed(x)), list(reversed(y)))

        self.T_points_x = x
        self.T_points_y = y
        self.T_coeffs = np.polyfit(interp_x_points, interp_y_points, deg)
        self.inv_T_coeffs = np.polyfit(interp_y_points,interp_x_points,deg)
        self.T_func = np.poly1d(self.T_coeffs)
        self.inv_T_func = np.poly1d(self.inv_T_coeffs)
        self.T_sympy_expr = self._get_sympy_expr(self.SOC_sym, self.T_coeffs)

    def _get_sympy_expr(self, x_sym, coeffs):
        return sum([coeffs[i]*x_sym**(len(coeffs)-i-1) for i in range(len(coeffs))])

vcurve_2A_25C = VCurve(3350, 2.,{
"x": [0.,61.8375,185.5124,315.371,463.7809,575.0883,692.5795,810.0707,939.9293,1106.8905,1242.9329,1366.6078,1490.2827,1613.9576,1719.0813,1799.47,1879.8587,1997.3498,2083.9223,2182.8622,2263.2509,2356.0071,2442.5795,2560.0707,2702.2968,2825.9717,2924.9117,3017.6678,3091.8728,3147.5265,3203.1802,3252.6502,3295.9364,3333.0389,3350.],
"y": [4.07,4.0171,3.986,3.9704,3.9393,3.9081,3.8614,3.8224,3.7913,3.7523,3.7134,3.6822,3.6433,3.6121,3.5732,3.5576,3.5265,3.5031,3.4798,3.4564,3.4408,3.4252,3.4019,3.3707,3.3318,3.285,3.2227,3.1526,3.067,2.9813,2.8801,2.7788,2.662,2.5607,2.43]
},
{
"x": [24.6914,191.358,401.2346,666.6667,1030.8642,1500,2086.4198,2629.6296,2944.4444,3148.1481,3246.9136,3339.5062],
"y": [25,25.9375,26.25,26.875,26.875,26.875,27.1875,27.5,28.125,29.375,30,31.5625]
})



vcurve_4A_25C = VCurve(3350,4.,{
"x": [43.2862,154.5936,303.0035,432.8622,593.6396,742.0495,890.4594,1057.4205,1174.9117,1329.5053,1459.364,1589.2226,1731.4488,1873.6749,2034.4523,2207.5972,2337.4558,2479.682,2609.5406,2757.9505,2881.6254,2999.1166,3073.3216,3135.159,3196.9965,3252.6502,3295.9364],
"y": [3.9315,3.8925,3.8847,3.8614,3.8146,3.7757,3.7212,3.6822,3.6433,3.6044,3.5654,3.5265,3.4798,3.4408,3.3941,3.3551,3.324,3.285,3.2539,3.1994,3.1371,3.0514,2.9735,2.8879,2.771,2.6464,2.5218]
},
{
"x": [12.3457,209.8765,419.7531,672.8395,1006.1728,1388.8889,1691.358,2030.8642,2333.3333,2623.4568,2858.0247,3092.5926,3290.1235],
"y": [25.3125,27.5,28.75,30,30.3125,31.25,31.875,31.875,32.1875,33.125,33.75,35,37.5]
})

# no temp data
#vcurve_6A_25C = VCurve(3350,6.,{
#"x": [18.5185,67.9012,129.6296,216.0494,308.642,450.6173,604.9383,753.0864,888.8889,1061.7284,1314.8148,1685.1852,1944.4444,2166.6667,2351.8519,2574.0741,2722.2222,2839.5062,2969.1358,3080.2469,3154.321,3222.2222,3290.1235],
#"y": [3.8516,3.8125,3.8047,3.7969,3.7891,3.7656,3.7266,3.6875,3.6406,3.5938,3.5234,3.4063,3.3281,3.2734,3.2188,3.1563,3.1016,3.0625,2.9688,2.875,2.7813,2.6563,2.5234]
#},
#{"x":[], "y":[]}
#)

# no temp data
#vcurve_8A_25C = VCurve(3350,8.,{
#"x": [30.8642,117.284,209.8765,290.1235,376.5432,493.8272,611.1111,950.6173,1290.1235,1623.4568,1895.0617,2290.1235,2598.7654,2783.9506,2901.2346,2993.8272,3098.7654,3203.7037,3265.4321,3290.1235],
#"y": [3.75,3.7188,3.7188,3.7188,3.7109,3.6875,3.6563,3.5625,3.4688,3.3594,3.2734,3.1719,3.0781,3.0156,2.9531,2.8984,2.8125,2.6797,2.5703,2.5]
#},
#{"x":[], "y":[]}
#)

vcurve_4A_10C = VCurve(3350,4.,{
"x": [31.0284,99.2908,210.9929,328.9007,583.3333,788.1206,1265.9574,1818.2624,2128.5461,2401.5957,2724.2908,2804.9645,2885.6383,2953.9007,3003.5461,3078.0142,3127.6596,3183.5106],
"y": [3.7633,3.7398,3.7633,3.7633,3.732,3.6771,3.5517,3.3793,3.2853,3.2147,3.0972,3.0502,2.9953,2.9404,2.8777,2.7602,2.6426,2.5094]
},
{
"x": [6.2057,161.3475,366.1348,577.1277,850.1773,1197.695,1570.0355,1836.8794,2140.9574,2488.4752,2848.4043,3059.3972,3171.0993],
"y": [10.094,13.2288,15.4232,16.6771,17.6176,17.931,18.8715,19.185,19.8119,21.0658,22.3197,24.2006,26.0815]
})

vcurve_4A_0C = VCurve(3350,4.,{
"x": [43.4397,124.1135,242.0213,341.3121,496.4539,664.0071,1036.3475,1464.539,1979.6099,2296.0993,2587.766,2780.1418,2910.461,3003.5461,3071.8085],
"y": [3.5831,3.6144,3.6536,3.6693,3.6693,3.6458,3.5517,3.4185,3.2461,3.1442,3.0345,2.9248,2.8072,2.6661,2.5094]
},
{
"x": [6.2057,130.3191,254.4326,397.1631,558.5106,806.7376,1036.3475,1309.3972,1706.5603,2004.4326,2382.9787,2668.4397,2879.4326,2997.3404,3071.8085],
"y": [0.3762,3.8245,6.3323,7.8997,9.1536,9.7806,10.4075,10.4075,11.348,11.6614,13.2288,14.7962,16.0502,17.3041,18.8715]
})


vcurve_4A_neg10C = VCurve(3350,4.,{
"x": [31.0284,86.8794,167.5532,266.844,390.9574,552.305,707.4468,943.2624,1203.9007,1526.5957,1936.1702,2364.3617,2711.8794,2854.6099,2935.2837],
"y": [3.3558,3.4107,3.4812,3.5439,3.5674,3.5674,3.5439,3.4969,3.4185,3.3009,3.152,2.9796,2.7759,2.6426,2.5172]
},
{
"x": [18.617,130.3191,285.461,459.2199,632.9787,980.4965,1421.0993,1818.2624,2190.6028,2525.7092,2668.4397,2829.7872,2935.2837],
"y": [-9.0282,-4.0125,-0.5643,1.6301,2.2571,3.1975,3.8245,5.0784,6.3323,7.8997,8.8401,10.4075,11.9749]
})

curves = [vcurve_2A_25C, vcurve_4A_25C, vcurve_4A_10C, vcurve_4A_0C, vcurve_4A_neg10C]

SOC_sym = vcurve_2A_25C.SOC_sym
V_sym = vcurve_2A_25C.V_sym
OCV_sym = sp.Symbol('OCV', real=True)
cell_capacity_C_sym = sp.Symbol('cell_capacity_C', positive=True)
used_capacity_C_sym = sp.Symbol('used_capacity_C', real=True)
reserve_SOC = sp.Symbol('reserve_soc', positive=True)

R = 0.044
Vdrop = R*vcurve_2A_25C.current

# compute SOC from OCV
SOC_from_OCV = vcurve_2A_25C.soc_from_v(OCV_sym-Vdrop)#.subs({V_sym:OCV_sym-Vdrop})

# compute OCV from SOC
OCV_from_SOC = vcurve_2A_25C.v_from_soc(SOC_sym)+Vdrop

# project pack capacity from SOC and used capacity
pack_capacity_from_OCV_and_discharge = used_capacity_C_sym/(1-SOC_from_OCV)
pack_capacity_from_OCV_and_discharge_func = lambdify([OCV_sym,used_capacity_C_sym], pack_capacity_from_OCV_and_discharge)

# compute remaining
remaining_energy = sp.integrate(OCV_from_SOC, (SOC_sym, 0, SOC_sym))
remaining_energy_from_OCV = remaining_energy.subs({SOC_sym:SOC_from_OCV})

SOC_from_OCV_func = lambdify([OCV_sym], SOC_from_OCV)

remaining_energy_from_OCV_func = lambdify([OCV_sym], remaining_energy_from_OCV)


print(OCV_from_SOC)
print(sp.simplify(SOC_from_OCV))
print(SOC_from_OCV_func(4.1796875))

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    SOC = np.linspace(-0.1,1.1,1000)
    V = np.linspace(2.3,4.4)
    coeffs = [2.58785, 1.78815, -0.07095, -0.5894, 0.05735]
    #plt.plot(SOC, np.poly1d(list(reversed(coeffs)))(SOC))



    #plt.plot(interp_x_points, interp_y_points, marker='+', linestyle='')
    #plt.plot(interp_x_points, np.poly1d(np.polyfit(interp_x_points, interp_y_points, 9))(interp_x_points))

    print(list(zip(*list(reversed(list(zip(vcurve_2A_25C.v_points_x,vcurve_2A_25C.v_points_y+.044*2)))))))
    plt.plot(vcurve_2A_25C.v_points_x,vcurve_2A_25C.v_points_y+.044*2, marker='x', linestyle='')
    plt.plot(SOC, OCV_from_SOC(SOC))
    #plt.plot(SOC_from_OCV_func(V-0.044*2), V-0.044*2)
    plt.show()
