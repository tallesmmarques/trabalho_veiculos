import class_car as cp
from scipy.io import savemat

# for u in [0.05, 0.10, 0.15, 0.20, 0.25]:

# Testar 5 casos
# u = 0.05
# u = 0.10
# u = 0.15
# u = 0.20
u = 0.25

car = cp.CarCoppelia()
car.startMission()
name = int(u*100)

print(f"Início {name}")

while car.t < 15.0:
    # lê senores
    car.step()

    # atua
    if car.t < 2.0:
        car.setU(u)
    else:
        car.setU(0.0)

t = [traj['t'] for traj in car.traj]
v = [traj['v'] for traj in car.traj]
    
car.stopMission()

tmp = {f"t{name:02d}": t, f"v{name:02d}": v}
savemat(f"dados_0_{name:02d}.mat", tmp)

print(f"Fim {name}")
	
print('Terminou...')
