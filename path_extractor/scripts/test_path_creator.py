#Skrypt generuje prosta sciezke do testow path_extractora
#Domyslnie sciezka jest prosta, z parametrem 1 jest parabola, z parametrem 2 okregiem

import pickle
import numpy as np
import sys

data = {}
if len(sys.argv) == 1 or sys.argv[1] == '0':
    data['pathpoints'] = [
        (-5+0.2*i, 0)#straight line
        for i in range(50)#for line and parabola
    ]

elif sys.argv[1] == '1':
    data['pathpoints'] = [
        (-5+0.2*i, (-5+0.2*i)*(-5+0.2*i)/2.5-5)#parabola
        for i in range(50)#for line and parabola
    ]

elif sys.argv[1] == '2':
    data['pathpoints'] = [
        (3*np.sin(i/10.0), 3*np.cos(i/10.0))#circle
        for i in range(62)#for circle
    ]

with open('path_data.pkl', 'wb') as f:
    pickle.dump(data, f, 2)
