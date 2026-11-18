"""Independent development check: python compare_qp_scipy.py /path/to/qp_probe."""
import subprocess
import sys
import numpy as np
from scipy.optimize import minimize

rng = np.random.default_rng(731)
cases = []
for _ in range(120):
    lower = rng.uniform(10., 150., 3)
    upper = lower + rng.uniform(50., 500., 3)
    witness = rng.uniform(lower, upper)
    a = rng.normal(size=(2, 3))
    b = a @ witness + rng.uniform(0., 50., 2)
    h = 10.**rng.uniform(-4., 1., 3)
    desired = rng.uniform(lower - 100., upper + 100.)
    g = -h * desired
    cases.append((h, g, lower, upper, a, b, witness))
lines = [' '.join(map(str, np.concatenate([h, g, lower, upper, a.ravel(), b])))
         for h, g, lower, upper, a, b, _ in cases]
proc = subprocess.run([sys.argv[1]], input='\n'.join(lines)+'\n', text=True, capture_output=True, check=True)
answers = [np.fromstring(line, sep=' ') for line in proc.stdout.splitlines()]
assert len(answers) == len(cases)
for i, ((h, g, lower, upper, a, b, witness), answer) in enumerate(zip(cases, answers)):
    assert answer[0] == 1, (i, 'feasible instance rejected')
    x = answer[1:]
    width = upper - lower
    scale = max(1., np.max(h * width**2))
    def cost(y):
        z = lower + width*y
        return (.5*np.dot(h*z, z) + g@z)/scale
    def jac(y):
        return width*(h*(lower+width*y)+g)/scale
    result = minimize(cost, (witness-lower)/width, jac=jac, method='SLSQP',
        bounds=[(0., 1.)]*3, constraints={'type':'ineq',
            'fun':lambda y: (b-a@(lower+width*y))/np.linalg.norm(a,axis=1),
            'jac':lambda y: -a*width/np.linalg.norm(a,axis=1)[:,None]},
        options={'ftol':1e-12,'maxiter':500})
    assert result.success, (i, result.message)
    assert np.max(a@x-b)<1e-6 and np.min(x-lower)>-1e-6 and np.max(x-upper)<1e-6
    assert abs(cost((x-lower)/width)-result.fun)<1e-7, (i, x, result.x, result.fun)
print(f'{len(cases)} QP optima agree with independent SciPy SLSQP reference')
