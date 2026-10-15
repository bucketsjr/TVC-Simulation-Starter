function check_demo()
%CHECK_DEMO Local MATLAB verification. Run this before trusting edited results.
% Copyright (c) 2026 James Bourke. MIT License.
p=tvc_parameters();
A=[0 1;-p.b*p.Kp -p.b*p.Kd];
assert(all(real(eig(A))<0),'PD poles should be stable.');
r=tvc_attitude(p,'PD',true);
exact=zeros(numel(r.t),2);
for k=1:numel(r.t)
    exact(k,:)=(expm(A*r.t(k))*[p.theta0;p.omega0])';
end
assert(max(max(abs(exact-[r.theta r.omega])))<1e-7,'Analytical solution mismatch.');
assert(abs(r.theta(end))<0.01*pi/180,'Nominal PD recovery failed.');
assert(max(abs(r.delta))<=p.delta_max+1e-12,'Command exceeded angle limit.');
off=tvc_attitude(p,'off',true);
assert(max(abs(off.theta-(p.theta0+p.omega0*off.t)))<1e-8,'Unforced motion mismatch.');
f=tvc_flight(p,'PD');o=tvc_flight(p,'off');
assert(f.apogee>o.apogee,'Nominal toy-model PD benefit disappeared.');
p.theta0=0;p.omega0=0;
f=tvc_flight(p,'PD');v=tvc_flight(p,'ideal');
assert(isequal(size(f.y),size(v.y)) && max(abs(f.y(:)-v.y(:)))<1e-8, ...
    'Zero-disturbance PD should match ideal baseline.');
fprintf('MATLAB analytical and physical sanity checks passed.\n');
end
