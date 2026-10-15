"""Independent Python reproduction of the educational MATLAB equations.
Copyright (c) 2026 James Bourke. MIT License. No NAYA data.
Needs numpy, scipy, matplotlib. Run from any working directory.
"""
from pathlib import Path
import csv, json
import numpy as np
from scipy.integrate import solve_ivp

ROOT = Path(__file__).resolve().parents[1]

def parameters():
    return dict(mass=.75, inertia=.028, thrust=12., arm=.16, zeta=.8,
                settling_target=1.8, delta_max=np.deg2rad(7.),
                theta0=np.deg2rad(8.), omega0=np.deg2rad(4.),
                burn=4., gravity=9.81, density=1.225, cd_area=.003,
                initial_height=2., attitude_end=6., flight_end=20., sample_dt=.01, max_step=.01)

def gains(p):
    b = p['thrust']*p['arm']/p['inertia']
    wn = 4/(p['zeta']*p['settling_target'])
    return wn*wn/b, 2*p['zeta']*wn/b, b

def command(theta, omega, p, mode):
    kp, kd, _ = gains(p)
    if mode in ('off','ideal'):
        return np.zeros_like(theta)
    return np.clip(-kp*theta - (kd*omega if mode == 'PD' else 0),
                   -p['delta_max'], p['delta_max'])

def attitude(p, mode, linear=False):
    _, _, b = gains(p)
    def rhs(t, y):
        d = command(y[0], y[1], p, mode)
        return [y[1], b*(d if linear else np.sin(d))]
    t = np.arange(0, p['attitude_end']+p['sample_dt']/2, p['sample_dt'])
    sol = solve_ivp(rhs, [0,t[-1]], [p['theta0'],p['omega0']], t_eval=t,
                    rtol=1e-10, atol=1e-12, max_step=p['max_step'])
    assert sol.success
    return sol.t, sol.y.T, command(sol.y[0],sol.y[1],p,mode)

def flight(p, mode):
    """State = [theta, omega, x, z, vx, vz]; thrust angle = theta-delta."""
    def rhs(t,y,powered):
        theta, omega, x, z, vx, vz = y
        T = p['thrust'] if powered else 0.
        delta = command(theta,omega,p,mode) if powered else 0.
        speed = np.hypot(vx,vz)
        drag = .5*p['density']*p['cd_area']*speed
        alpha = theta-delta
        return [omega, T*p['arm']/p['inertia']*np.sin(delta), vx, vz,
                (T*np.sin(alpha)-drag*vx)/p['mass'],
                (T*np.cos(alpha)-drag*vz)/p['mass']-p['gravity']]
    def ground(t,y): return y[3]
    ground.terminal = True; ground.direction = -1
    initial = [0. if mode=='ideal' else p['theta0'],
               0. if mode=='ideal' else p['omega0'],0.,p['initial_height'],0.,0.]
    # Split at burnout so a solver step cannot smear the thrust discontinuity.
    times=[]; states=[]
    for start, end, powered in [(0.,p['burn'],True),(p['burn'],p['flight_end'],False)]:
        t = np.linspace(start,end,round((end-start)/p['sample_dt'])+1)
        sol = solve_ivp(lambda t,y:rhs(t,y,powered),[start,end],initial,
                        t_eval=t,events=ground,rtol=1e-10,atol=1e-12,max_step=p['max_step'])
        assert sol.success
        if times: times.extend(sol.t[1:]); states.extend(sol.y.T[1:])
        else: times.extend(sol.t); states.extend(sol.y.T)
        if len(sol.t_events[0]):
            times.append(sol.t_events[0][0]); states.append(sol.y_events[0][0]); break
        initial=sol.y[:,-1]
    return np.array(times),np.array(states)

def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,
                         'axes.spines.top':False,'axes.spines.right':False})
    p=parameters(); kp,kd,b=gains(p)
    colors={'off':'#9c5252','P':'#b17e26','PD':'#146b85','ideal':'#444444'}
    labels={'off':'TVC off','P':'P only','PD':'PD control','ideal':'Ideal vertical reference'}
    att={m:attitude(p,m) for m in ['off','P','PD']}
    fl={m:flight(p,m) for m in ['ideal','off','P','PD']}
    metrics=[]
    for m,(t,y,d) in att.items():
        bad=np.flatnonzero(np.abs(y[:,0])>.02*abs(p['theta0']))
        settled=0. if not len(bad) else (float(t[bad[-1]+1]) if bad[-1]<len(t)-1 else None)
        metrics.append(dict(mode=m,attitude_settle_2pct_s=settled,
            attitude_rms_deg=float(np.sqrt(np.mean(np.rad2deg(y[:,0])**2))),
            max_gimbal_deg=float(np.max(np.abs(np.rad2deg(d)))),
            apogee_m=float(np.max(fl[m][1][:,3])),
            burnout_height_m=float(np.interp(p['burn'],fl[m][0],fl[m][1][:,3]))))
    metrics.insert(0,dict(mode='ideal',attitude_settle_2pct_s=None,attitude_rms_deg=0.,
                          max_gimbal_deg=0.,apogee_m=float(np.max(fl['ideal'][1][:,3])),
                          burnout_height_m=float(np.interp(p['burn'],fl['ideal'][0],fl['ideal'][1][:,3]))))
    (ROOT/'results').mkdir(exist_ok=True); (ROOT/'figures').mkdir(exist_ok=True)
    with (ROOT/'results/reference_metrics.json').open('w') as f:
        json.dump(dict(provenance='Fresh generic simulation; independently executed in Python; not NAYA results',
                       parameters=p,Kp=kp,Kd=kd,b=b,metrics=metrics),f,indent=2)
    with (ROOT/'results/reference_metrics.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(metrics[0]));w.writeheader();w.writerows(metrics)
    for m,(t,y,d) in att.items():
        np.savetxt(ROOT/f'results/attitude_{m}.csv',np.column_stack([t,y,d]),delimiter=',',
                   header='time_s,angle_rad,rate_rad_s,gimbal_rad',comments='')
    for m,(t,y) in fl.items():
        np.savetxt(ROOT/f'results/flight_{m}.csv',np.column_stack([t,y]),delimiter=',',
                   header='time_s,angle_rad,rate_rad_s,x_m,z_m,vx_m_s,vz_m_s',comments='')
    fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
    for m,(t,y,d) in att.items():
        axs[0].plot(t,np.rad2deg(y[:,0]),label=labels[m],c=colors[m],lw=2)
        axs[1].plot(t,np.rad2deg(d),c=colors[m],lw=2)
    axs[0].axhline(0,color='.7',lw=.7);axs[0].legend(ncol=3,frameon=False)
    axs[0].set_ylabel('Attitude from vertical (deg)');axs[1].set_ylabel('Commanded gimbal (deg)')
    axs[1].set_xlabel('Time (s)');axs[1].set_ylim(-8,8)
    axs[1].axhline(7,c='.7',ls='--');axs[1].axhline(-7,c='.7',ls='--')
    for ax in axs: ax.grid(alpha=.18)
    fig.suptitle('From angle error to a damped response',weight='bold',y=.98)
    fig.text(.5,.015,'GENERIC SIMULATION  |  Nonlinear torque, perfect sensing, instantaneous actuation  |  Not NAYA test data',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(ROOT/'figures/attitude_response.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,5))
    for m,(t,y) in fl.items():
        axs[0].plot(t,y[:,3],label=labels[m],c=colors[m],lw=2,ls='--' if m=='ideal' else '-')
        axs[1].plot(y[:,2],y[:,3],c=colors[m],lw=2,ls='--' if m=='ideal' else '-')
    axs[0].axvline(p['burn'],c='.7',ls=':',label='Burnout')
    axs[0].set(xlabel='Time (s)',ylabel='Height above ground (m)')
    axs[1].set(xlabel='Horizontal position (m)',ylabel='Height above ground (m)')
    axs[0].legend(frameon=False,fontsize=9);[ax.grid(alpha=.18) for ax in axs]
    fig.suptitle('What attitude recovery changes in a toy flight',weight='bold')
    fig.text(.5,.015,'HYPOTHETICAL FLIGHT  |  Constant mass, constant powered thrust, simple drag, no aero moments  |  Not NAYA flight',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(ROOT/'figures/flight_comparison.png',dpi=180);plt.close(fig)
    print(json.dumps(metrics,indent=2))
if __name__=='__main__': main()
