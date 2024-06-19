import numpy as np

def fnc_STSMC(chi, r, Phi, sqrt_k1_STSMC, k2_STSMC, v_a_prior, dt):
    e = chi - r
    s = e

    sqrt_abs_s = np.sqrt(np.abs(s))

    sat_s = np.zeros((3,1))
    if abs(s[0]) > Phi[0]:
        sat_s[0] = np.sign(s[0])
    else:
        sat_s[0] = s[0] / Phi[0]

    if abs(s[1]) > Phi[1]:
        sat_s[1] = np.sign(s[1])
    else:
        sat_s[1] = s[1] / Phi[1]

    if abs(s[2]) > Phi[2]:
        sat_s[2] = np.sign(s[2])
    else:
        sat_s[2] = s[2] / Phi[2]

    d_v_a = -k2_STSMC @ sat_s
    v_a = v_a_prior + d_v_a * dt
    STSMC_signal = -sqrt_k1_STSMC @ np.multiply(sqrt_abs_s, sat_s) + v_a

    return STSMC_signal, v_a


def fnc_f(chi):
    x, y, z = chi[0], chi[1], chi[2]

    f1 = np.array([0.00462508752*x**2 + 0.00131465836*x*y - 0.0142143732*x*z - 0.0042610791*y**2 + 0.00139716813*y*z + 0.00172011516*z**2,
          0.00182551438*x**2 - 0.00684164917*x*y + 0.000394107486*x*z - 0.00115148951*y**2 - 0.0184253657*y*z + 0.00636863261*z**2,
          0.00021874013*x**2 - 0.000158299422*x*y + 0.000408788577*x*z - 0.000227911296*y**2 - 0.00177517257*y*z - 0.00907110732*z**2]).reshape(-1,1)

    A = np.array([[-1.5510, 0.0256, -0.1285],
         [0.0225, -1.6231, 0.2889],
         [0.1676, 0, -1.7543]])

    f = np.dot(A, chi) + f1

    return f

def tau_sld(v,u_hat,chi,Gamma):

    s1 = u_hat[0]
    s2 = u_hat[1]
    s3 = u_hat[2]
    x = chi[0]
    y = chi[1]
    z = chi[2]

    dh_du = np.array([
        [0.00600873368 * s1 - 0.000375934751 * s2 + 0.000277284774 * s3 + 0.00173092916 * x + 0.00291715264 * y - 0.0142394595 * z - 0.479621808, 0.00495171736 * s2 - 0.000375934751 * s1 - 0.0000523625768 * s3 + 0.00299632591 * x - 0.00248808349 * y - 0.0113808922 * z - 0.405123688, 0.000277284774 * s1 - 0.0000523625768 * s2 - 0.0078037221 * s3 - 0.00527578274 * x - 0.000873893696 * y + 0.0234114079 * z + 1.0547403],
        [0.000123846048 * s3 - 0.000407297045 * s2 - 0.00853242928 * s1 + 0.00177241117 * x - 0.00228205009 * y + 0.023309966 * z + 0.839276465, 0.00921987154 * s2 - 0.000407297045 * s1 - 0.000434411748 * s3 + 0.000398386786 * x - 0.000899690906 * y - 0.023332838 * z - 0.903787295, 0.000123846048 * s1 - 0.000434411748 * s2 - 0.00048423827 * s3 - 0.00159360508 * x + 0.00406241707 * y - 0.000543430891 * z - 0.00929028925],
        [0.000725837376 * s1 - 0.000280116518 * s2 - 0.000477654246 * s3 + 0.00392727888 * x - 0.00594151866 * y + 0.000710971314 * z + 0.15035566, 0.000902969642 * s2 - 0.000280116518 * s1 + 0.0000659375225 * s3 + 0.00351302114 * x + 0.006446466 * y - 0.00183034739 * z + 0.169452516, 0.0000659375225 * s2 - 0.000477654246 * s1 + 0.0000490147836 * s3 - 0.00649083508 * x - 0.00000910474099 * y + 0.00588586873 * z + 0.346900453]
    ]).squeeze()
    h_u_hat = np.array([
        1.0547403 * s3 - 0.405123688 * s2 - 0.479621808 * s1 - 0.000375934751 * s1 * s2 + 0.000277284774 * s1 * s3 - 0.0000523625768 * s2 * s3 + 0.00173092916 * s1 * x + 0.00299632591 * s2 * x - 0.00527578274 * s3 * x + 0.00291715264 * s1 * y - 0.00248808349 * s2 * y - 0.000873893696 * s3 * y - 0.0142394595 * s1 * z - 0.0113808922 * s2 * z + 0.0234114079 * s3 * z + 0.00300436684 * s1**2 + 0.00247585868 * s2**2 - 0.00390186105 * s3**2,
        0.839276465 * s1 - 0.903787295 * s2 - 0.00929028925 * s3 - 0.000407297045 * s1 * s2 + 0.000123846048 * s1 * s3 - 0.000434411748 * s2 * s3 + 0.00177241117 * s1 * x + 0.000398386786 * s2 * x - 0.00159360508 * s3 * x - 0.00228205009 * s1 * y - 0.000899690906 * s2 * y + 0.00406241707 * s3 * y + 0.023309966 * s1 * z - 0.023332838 * s2 * z - 0.000543430891 * s3 * z - 0.00426621464 * s1**2 + 0.00460993577 * s2**2 - 0.000242119135 * s3**2,
        0.15035566 * s1 + 0.169452516 * s2 + 0.346900453 * s3 - 0.000280116518 * s1 * s2 - 0.000477654246 * s1 * s3 + 0.0000659375225 * s2 * s3 + 0.00392727888 * s1 * x + 0.00351302114 * s2 * x - 0.00649083508 * s3 * x - 0.00594151866 * s1 * y + 0.006446466 * s2 * y - 0.00000910474099 * s3 * y + 0.000710971314 * s1 * z - 0.00183034739 * s2 * z + 0.00588586873 * s3 * z + 0.000362918688 * s1**2 + 0.000451484821 * s2**2 + 0.0000245073918 * s3**2
    ]).reshape(-1,1)

    v_err = v - h_u_hat

    # Calculate the sgn_aprx vector
    # sgn_aprx = np.arctan(v_err * 2) * 2 / np.pi
    sgn_aprx = 2 / (np.exp(-v_err * 2) + 1) - 1

    # Calculate the pseudo-inverse of dh_du
    dh_du_inv = np.linalg.pinv(dh_du)
    

    # Calculate the u_hat_dot vector
    u_hat_dot = np.dot(np.dot(dh_du_inv, Gamma), sgn_aprx)

    # Calculate the determinant of dh_du_inv
    dh_du_inv_det = np.linalg.det(dh_du_inv)  # To evaluate inversibility

    return u_hat_dot, dh_du_inv_det

def fnc_f2(u, psi):
    s1 = u[0]
    s2 = u[1]
    s3 = u[2]
    x = psi[0]
    y = psi[1]
    z = psi[2]

    f2 = np.array([
        0.00173092916 * s1 * x + 0.00299632591 * s2 * x - 0.00527578274 * s3 * x + 0.00291715264 * s1 * y - 0.00248808349 * s2 * y - 0.000873893696 * s3 * y - 0.0142394595 * s1 * z - 0.0113808922 * s2 * z + 0.0234114079 * s3 * z,
        0.00177241117 * s1 * x + 0.000398386786 * s2 * x - 0.00159360508 * s3 * x - 0.00228205009 * s1 * y - 0.000899690906 * s2 * y + 0.00406241707 * s3 * y + 0.023309966 * s1 * z - 0.023332838 * s2 * z - 0.000543430891 * s3 * z,
        0.00392727888 * s1 * x + 0.00351302114 * s2 * x - 0.00649083508 * s3 * x - 0.00594151866 * s1 * y + 0.006446466 * s2 * y - 0.00000910474099 * s3 * y + 0.000710971314 * s1 * z - 0.00183034739 * s2 * z + 0.00588586873 * s3 * z
    ]).reshape(-1,1)

    return f2


def fnc_g(u):
    s1 = u[0]
    s2 = u[1]
    s3 = u[2]

    g = np.array([
        0.00300436684 * s1**2 - 0.000375934751 * s1 * s2 + 0.000277284774 * s1 * s3 + 0.00247585868 * s2**2 - 0.0000523625768 * s2 * s3 - 0.00390186105 * s3**2,
        -0.00426621464 * s1**2 - 0.000407297045 * s1 * s2 + 0.000123846048 * s1 * s3 + 0.00460993577 * s2**2 - 0.000434411748 * s2 * s3 - 0.000242119135 * s3**2,
        0.000362918688 * s1**2 - 0.000280116518 * s1 * s2 - 0.000477654246 * s1 * s3 + 0.000451484821 * s2**2 + 0.0000659375225 * s2 * s3 + 0.0000245073918 * s3**2
    ]).reshape(-1,1)

    return g

def tau_fnc(v,u,psi,Gamma):
    s1 = u[0]
    s2 = u[1]
    s3 = u[2]
    x = psi[0]
    y = psi[1]
    z = psi[2]

    dh_du = np.array([
        [0.00600873368 * s1 - 0.000375934751 * s2 + 0.000277284774 * s3 + 0.00173092916 * x + 0.00291715264 * y - 0.0142394595 * z - 0.479621808, 0.00495171736 * s2 - 0.000375934751 * s1 - 0.0000523625768 * s3 + 0.00299632591 * x - 0.00248808349 * y - 0.0113808922 * z - 0.405123688, 0.000277284774 * s1 - 0.0000523625768 * s2 - 0.0078037221 * s3 - 0.00527578274 * x - 0.000873893696 * y + 0.0234114079 * z + 1.0547403],
        [0.000123846048 * s3 - 0.000407297045 * s2 - 0.00853242928 * s1 + 0.00177241117 * x - 0.00228205009 * y + 0.023309966 * z + 0.839276465, 0.00921987154 * s2 - 0.000407297045 * s1 - 0.000434411748 * s3 + 0.000398386786 * x - 0.000899690906 * y - 0.023332838 * z - 0.903787295, 0.000123846048 * s1 - 0.000434411748 * s2 - 0.00048423827 * s3 - 0.00159360508 * x + 0.00406241707 * y - 0.000543430891 * z - 0.00929028925],
        [0.000725837376 * s1 - 0.000280116518 * s2 - 0.000477654246 * s3 + 0.00392727888 * x - 0.00594151866 * y + 0.000710971314 * z + 0.15035566, 0.000902969642 * s2 - 0.000280116518 * s1 + 0.0000659375225 * s3 + 0.00351302114 * x + 0.006446466 * y - 0.00183034739 * z + 0.169452516, 0.0000659375225 * s2 - 0.000477654246 * s1 + 0.0000490147836 * s3 - 0.00649083508 * x - 0.00000910474099 * y + 0.00588586873 * z + 0.346900453]
    ]).squeeze()
        
    M = dh_du.T

    h_u_hat = np.array([
        1.0547403 * s3 - 0.405123688 * s2 - 0.479621808 * s1 - 0.000375934751 * s1 * s2 + 0.000277284774 * s1 * s3 - 0.0000523625768 * s2 * s3 + 0.00173092916 * s1 * x + 0.00299632591 * s2 * x - 0.00527578274 * s3 * x + 0.00291715264 * s1 * y - 0.00248808349 * s2 * y - 0.000873893696 * s3 * y - 0.0142394595 * s1 * z - 0.0113808922 * s2 * z + 0.0234114079 * s3 * z + 0.00300436684 * s1**2 + 0.00247585868 * s2**2 - 0.00390186105 * s3**2,
        0.839276465 * s1 - 0.903787295 * s2 - 0.00929028925 * s3 - 0.000407297045 * s1 * s2 + 0.000123846048 * s1 * s3 - 0.000434411748 * s2 * s3 + 0.00177241117 * s1 * x + 0.000398386786 * s2 * x - 0.00159360508 * s3 * x - 0.00228205009 * s1 * y - 0.000899690906 * s2 * y + 0.00406241707 * s3 * y + 0.023309966 * s1 * z - 0.023332838 * s2 * z - 0.000543430891 * s3 * z - 0.00426621464 * s1**2 + 0.00460993577 * s2**2 - 0.000242119135 * s3**2,
        0.15035566 * s1 + 0.169452516 * s2 + 0.346900453 * s3 - 0.000280116518 * s1 * s2 - 0.000477654246 * s1 * s3 + 0.0000659375225 * s2 * s3 + 0.00392727888 * s1 * x + 0.00351302114 * s2 * x - 0.00649083508 * s3 * x - 0.00594151866 * s1 * y + 0.006446466 * s2 * y - 0.00000910474099 * s3 * y + 0.000710971314 * s1 * z - 0.00183034739 * s2 * z + 0.00588586873 * s3 * z + 0.000362918688 * s1**2 + 0.000451484821 * s2**2 + 0.0000245073918 * s3**2
    ]).reshape(-1,1)

    u_hat_d = Gamma @ M @ (v-h_u_hat)
    return u_hat_d


import numpy as np

def proj(u_hat, tau):
    u_min = np.array([-160, -160, -160]).reshape(-1,1)
    u_max = np.array([20, 20, 20]).reshape(-1,1)
    p = 0.5 * (u_min + u_max)
    q = 0.5 * (u_max - u_min)
    Q = np.diag(q.squeeze())
    epsilon = 0.1

    h = (1 + epsilon) * np.dot(np.dot((u_hat - p).T, np.linalg.inv(np.dot(Q, Q))), (u_hat - p)) - 1
    grad_h = 2 * (1 + epsilon) * np.dot(np.linalg.inv(np.dot(Q, Q)), (u_hat - p))

    c = min(1, h / epsilon)
    if h < 0:
        u_hat_dot_proj = tau
    elif h >= 0 and np.dot(grad_h.T, tau) <= 0:
        u_hat_dot_proj = tau
    else:
        grad_h_unit = grad_h / np.linalg.norm(grad_h)
        proj_1 = np.dot(grad_h_unit, np.dot(grad_h_unit.T, tau))
        proj_2 = tau - proj_1 * h
        u_hat_dot_proj = proj_2
    
    return u_hat_dot_proj


B1 = np.array([[-0.4796, -0.4051, 1.0547],
                [0.8393, -0.9038, -0.0093],
                [0.1504, 0.1695, 0.3469]])