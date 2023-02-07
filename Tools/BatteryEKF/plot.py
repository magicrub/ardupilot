#!/usr/bin/env python3
import scipy.io
import sys
import pickle
import matplotlib.pyplot as plt
import numpy as np
import glob
import sympy as sp
from matplotlib.widgets import MultiCursor
import json

def plot_state(name, label=None):
    plt.plot(data["t"], data[name], label=label)
    plt.fill_between(data["t"], data[name]-data["stddev"][name],data[name]+data["stddev"][name], alpha=.2)
    plt.title(name)

def deep_numpify(d):
    for k,v in d.items():
        if type(v) == list:
            #d[k] = np.asarray([x if x is not None else -1 for x in v])
            d[k] = np.asarray(v)#[x if x is not None else -1 for x in v])

        if type(v) == dict:
            deep_numpify(v)

def print_log_info(fn,data):
    has_nan = False
    for k,v in data.items():
        if type(v) == list and None in v:
            has_nan=True
            break

    n_samples = len(data["volt"])
    dt_max = max(data["dt"])
    t_max = max(data["t"])

    print('### Log %s ###' % (fn,))
    print('    File %s' % (data["filename"],))
    print('    n_samples=%u has_nan=%s dt_max=%f t_max=%f' % (n_samples,has_nan,dt_max,t_max))

plt.gcf().set_tight_layout(True)

def SOC_from_OCV(OCV):
    return -3.34159974198742*OCV**8 + 87.6766752425665*OCV**7 - 999.883264417696*OCV**6 + 6472.19014297556*OCV**5 - 26003.8684354005*OCV**4 + 66397.9196658339*OCV**3 - 105211.043879198*OCV**2 + 94583.5821042438*OCV - 36933.7424310365

#count = 0
for fn in sys.argv[1:]:
    #print(count)
    #count += 1
    with open(fn) as f:
        try:
            data = json.load(f)
        except:
            print("%s - bad data" % (fn,))
            continue

    deep_numpify(data)

    if len(sys.argv) > 2:
        if len(data["volt"]) <= 100 or data["volt"][100] > 26/6. or max(data["currtot"]) < 10000 or max(data["NIS"]) < 1:
            continue

    data["currtot"] /= 16.
    data["curr"] /= 16.
    data["volt"] /= 6.
    data["R"] = data["R"]*16./6.

    #if None in data["NIS"]:
        #print("%s - contains NaN" % (fn,))
        #continue
    #if max(data["NIS"]) > 1:
        #print("%s - max NIS %f" % (fn, max(data["NIS"])))

    #continue


    print_log_info(fn,data)


    ax1 = plt.subplot(331)
    plt.plot(data["t"], data["curr"])
    plt.title("I")

    plt.subplot(332,sharex=ax1)
    plt.plot(data["t"], data["volt"])
    plt.title("V")
    plt.plot(data["t"], data["volt"] + data["curr"]*data["R"], label="AP OCV")
    plt.plot(data["t"], data["volt"]+data["curr"]*data["R0"]+data["V1"]+data["V2"], label="OCV")

    plt.subplot(333,sharex=ax1)
    #plt.plot(data["t"], data["y"], label="y")
    plt.plot(data["t"], data["NIS"])
    plt.title("NIS")

    plt.subplot(334,sharex=ax1)
    plot_state("SOC", label="from EKF")
    plt.plot(data["t"], 1-(data["currtot"]*3.6/data["conf"]["Q"]*data["SOH_inv"]), label="from Itot/(Q*SOH)")
    plt.plot(data["t"], SOC_from_OCV(data["volt"]+data["curr"]*data["R0"]+data["V1"]+data["V2"]), label="from OCV")

    plt.subplot(335,sharex=ax1)
    soh = 1/data["SOH_inv"]
    soh_1sigma_low = soh-data["stddev"]["SOH_inv"]*soh**2
    soh_1sigma_high = soh+data["stddev"]["SOH_inv"]*soh**2
    plt.plot(data["t"], soh)
    plt.fill_between(data["t"], soh_1sigma_low, soh_1sigma_high, alpha=.2)
    plt.title("SOH")

    plt.subplot(336,sharex=ax1)
    plot_state("E")

    plt.subplot(337,sharex=ax1)
    plot_state("R0")
    plt.subplot(338,sharex=ax1)
    plot_state("R1")
    plt.subplot(339,sharex=ax1)
    plot_state("R2")
    #plt.plot(data["t"], data["R0"]+data["R1"]+data["R2"], label="R0+R1+R2")
    #plt.plot(data["t"], data["R"], label="AP R")

#print(plt.subplots()[0].get_axes())
ax1 = plt.subplot(331)
plt.legend(loc='upper right')
plt.subplot(332,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(333,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(334,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(335,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(336,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(337,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(338,sharex=ax1)
plt.legend(loc='upper right')
plt.subplot(339,sharex=ax1)
plt.legend(loc='upper right')

cursor = MultiCursor(None, plt.gcf().get_axes(), color='black', lw=1)

plt.show()
