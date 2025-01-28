from math import sin,cos,tan,atan,sqrt
# H = Starting Angle, V = Velocity, s = Steering Angle, L=wheelbase, dt = Change In Time
def antiderivative_x(theta,V,s,L,dt):
    H = theta-atan(tan(s)/2)
    upper_bound = L*sqrt(tan(s)**2+4)*sin(((2*V*tan(s)*dt)/(L*sqrt(tan(s)**2+4)))+atan(tan(s)/2)+H) / (2*tan(s))
    lower_bound = L*sqrt(tan(s)**2+4)*sin(((2*V*tan(s)*0)/(L*sqrt(tan(s)**2+4)))+atan(tan(s)/2)+H) / (2*tan(s))
    x = upper_bound-lower_bound
    return x

def antiderivative_y(theta,V,s,L,dt):
    H = theta-atan(tan(s)/2)
    upper_bound = -L*sqrt(tan(s)**2+4)*cos(((2*V*tan(s)*dt)/(L*sqrt(tan(s)**2+4)))+atan(tan(s)/2)+H) / (2*tan(s))
    lower_bound = -L*sqrt(tan(s)**2+4)*cos(((2*V*tan(s)*0)/(L*sqrt(tan(s)**2+4)))+atan(tan(s)/2)+H) / (2*tan(s))
    y = upper_bound-lower_bound
    return y