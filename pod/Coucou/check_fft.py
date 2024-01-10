#!/usr/bin/env python

import json
from pylab import *

def load_json(path):
    ret = None
    with open(path, "r") as handle:
        ret = json.load(handle)
    return ret

data = load_json("foo.json")


xxs = array(data["xxs"])
yys = array(data["yys"])

xxs = xxs[:,0] + 1j * xxs[:,1]
yys = yys[:,0] + 1j * yys[:,1]
zzs = fft(xxs).conj()

error = np.abs(yys - zzs).max()
print(f"error {error}")

fig = figure()
axes = fig.subplots(2,2)
axes[0][0].plot(xxs.real, c="r", label="rx")
axes[0][1].plot(xxs.imag, c="g", label="ix")
axes[1][0].plot(yys.real, c="b", label="ry")
axes[1][1].plot(yys.imag, c="k", label="iy")
axes[1][0].plot(zzs.real, c="y", label="rz")
axes[1][1].plot(zzs.imag, c="c", label="iz")
fig.legend()

show()