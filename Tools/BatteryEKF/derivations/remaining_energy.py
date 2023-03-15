from common import *

soh = Matrix([1/SOH_inv])

G = soh.jacobian(x)

soh_sigma = sqrt((G*P*G.T)[0])
print(soh, soh_sigma)

remaining_energy = Matrix([integrate(OCV_from_SOC(SOC, temp_C), (SOC, 0, SOC)) * Q / SOH_inv])

G = remaining_energy.jacobian(x)

remaining_energy_sigma = sqrt((G*P*G.T)[0])

subs = {integrate(OCV_from_SOC(SOC, temp_C), (SOC, 0, SOC)): Function("_model.OCV_from_SOC_integral")(SOC, temp_C)}

remaining_energy = remaining_energy.xreplace(subs)
remaining_energy_sigma = simplify(remaining_energy_sigma.xreplace(subs))

print(ccode(remaining_energy[0]))
print('')

print(ccode(remaining_energy_sigma))
