import numpy as np
from control.matlab import dare

def get_params(A, b, c, Q, R, N):

    [P,_,_] = dare(A, b, c.T*Q*c, R)
    K = (R + b.T*P*b).I*(b.T*P*A)

    f = np.ones((1,N))

    for i in range(N):
        f[0, i] = (R+b.T*P*b).I*b.T*(((A-b*K).T)**i)*c.T*Q

    return (K, f)

if __name__ == '__main__':

    ZMP_pos = np.array([[0.0, 0.0],
                        [0.5, 0.5],
                        [1.0, 0.2],
                        [2.0, 1.2],
                        [2.5, 0.5],
                        [3.0, 1.5]])

    z_COM = 0.5
    g = 9.81
    dt = 0.01
    step_time = 0.7
    preview_time_window = 1.0
    num_steps = len(ZMP_pos)

    t_simulation = num_steps*step_time - preview_time_window - dt

    N_preview = int(preview_time_window/dt)
    N_simulation = int(t_simulation/dt)

    ZMP_x_ref_with_time = []
    ZMP_y_ref__with_time = []

    i = 0
    for t in  np.arange(0, num_steps*step_time, dt):
        ZMP_x_ref_with_time.append(ZMP_pos[i,0])
        ZMP_y_ref__with_time.append(ZMP_pos[i,1])

        if (t != 0) and (t%step_time < 1e-6):
            i += 1

    A = np.mat(([1, dt, dt**2/2],
                [0, 1, dt],
                [0, 0, 1]))

    b = np.mat((dt**3/6, dt**2/2, dt)).T

    c = np.mat((1, 0, -z_COM/g))

    Q = 1
    R = 0.0001

    K, f = get_params(A, b, c, Q, R, N_preview)

    ux = np.asmatrix(np.zeros((N_simulation, 1)))
    uy = np.asmatrix(np.zeros((N_simulation, 1)))
    COM_x = np.asmatrix(np.zeros((3, N_simulation+1)))
    COM_y = np.asmatrix(np.zeros((3, N_simulation+1)))

    # record data for plot
    COM_x_record = []
    COM_y_record = []
    ZMP_x_record = []
    ZMP_y_record = []

    for k in range(N_simulation):
        ZMP_x_preview = np.asmatrix(ZMP_x_ref_with_time[k:k+N_preview]).T
        ZMP_y_preview = np.asmatrix(ZMP_y_ref__with_time[k:k+N_preview]).T

        # update ZMP
        ZMP_x_next = c*COM_x[:,k]
        ZMP_y_next = c*COM_y[:,k]
        ZMP_x_record.append(ZMP_x_next[0,0])
        ZMP_y_record.append(ZMP_y_next[0,0])

        # update u
        ux[k] = -K*COM_x[:, k] + f*ZMP_x_preview
        uy[k] = -K*COM_y[:, k] + f*ZMP_y_preview

        # update COM state
        COM_x[:,k+1] = A*COM_x[:, k] + b*ux[k]
        COM_y[:,k+1] = A*COM_y[:, k] + b*uy[k]
        COM_x_record.append(COM_x[0,k])
        COM_y_record.append(COM_y[0,k])

    import matplotlib.pyplot as plt
    plt.figure()
    plt.title("Preview Control")
    plt.subplot(3,1,1)
    plt.plot(ZMP_x_ref_with_time, 'g--', label='ZMP_x_ref')
    plt.plot(ZMP_x_record, 'b', label='ZMP_x')
    plt.plot(COM_x_record, 'r--', label='COM_x')
    plt.legend()
    plt.subplot(3,1,2)
    plt.plot(ZMP_y_ref__with_time, 'g--', label='ZMP_y_ref')
    plt.plot(ZMP_y_record, 'b', label='ZMP_y')
    plt.plot(COM_y_record, 'r--', label='COM_y')
    plt.legend()
    plt.subplot(3,1,3)
    plt.plot(ZMP_x_ref_with_time, ZMP_y_ref__with_time, 'g--', label='ZMP_ref')
    plt.plot(ZMP_x_record, ZMP_y_record, 'b', label='ZMP')
    plt.plot(COM_x_record, COM_y_record, 'r--', label='COM')
    plt.legend()

    plt.show()
