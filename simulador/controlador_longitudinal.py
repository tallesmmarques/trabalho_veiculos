import class_car as cp
from scipy.io import savemat
import matplotlib.pyplot as plt
plt.ion()
plt.figure(1)

car = cp.CarCoppelia()
car.startMission()

set_point_long = 2

last_e = 0
last_u = 0

while car.t < 7.0:
    # lê senores
    car.step()

	# seta direcao
    delta = -2.0*(0.0 - car.p[0]) + 5.0*(0.0 - car.w)
    car.setSteer(-delta)

    # atua
    v, w = car.getVel()
    error = set_point_long - v
    u = last_u + 0.624*error - 0.625*last_e
    car.setU(u)

    # atualiza valores anteriores
    last_e = error
    last_u = u
    
    plt.clf()
    t = [traj['t'] for traj in car.traj]
    v = [traj['v'] for traj in car.traj]
    plt.plot(t,v)
    plt.show()
    plt.pause(0.01)

car.stopMission()

# tmp = {f"t{name:02d}": t, f"v{name:02d}": v}
# savemat(f"dados_0_{name:02d}.mat", tmp)
	
print('Terminou...')
