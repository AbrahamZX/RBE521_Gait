

def generate_leg_positions(t):
    Lstep = .05  # m
    FootHeight = .05  # m
    t1 = 2.5  # seconds
    t2 = 5  # seconds
    t3 = 7.5  # seconds
    t4 = 10  # seconds

    # x and z positions for gait based on NaoDynamics paper
    #  Dynamic Modeling and Control Study of the NAO Biped Robot with
    #  Improved Trajectory Planning by Ehsan Hasemi
    if t % t4 < t1:
        xr = -Lstep
        zr = FootHeight
        xl = -1.5144*t**3 + 1.5855*t**2 - 0.2334*t - 0.0404
        zl = 8.1169*t**3 - 9.8338*t**2 + 3.575*t - 0.34347
    elif t % t4 < t2:
        xr = -1.2848*t**3 + 1.3202*t**2 - 0.1938*t - 0.0435
        zr = -19.724*t**3 + 10.2116*t**2 - 1.2359*t + 0.0412
        xl = Lstep
        zl = FootHeight
    elif t % t4 < t3:
        xr = -1.5144*t**3 + 1.5855*t**2 - 0.2334*t - 0.0404
        zr = 8.1169*t**3 - 9.8338*t**2 + 3.575*t - 0.34347
        xl = -Lstep
        zl = FootHeight
    else:
        xr = Lstep
        zr = FootHeight
        xl = -1.5144*t**3 + 1.5855*t**2 - 0.2334*t - 0.0404
        zl = 8.1169*t**3 - 9.8338*t**2 + 3.575*t - 0.34347
    return xr, zr, xl, zl
