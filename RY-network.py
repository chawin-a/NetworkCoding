from __future__ import print_function
from ortools.linear_solver import pywraplp

def RYNetwork(v1, v2):
    solver = pywraplp.Solver('RY-Network', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

    V = ['k1', 'k2', 'k3', 'e', 'R1', 'R2']
    v = [None] * 6
    for i in range(5):
        v[i] = solver.NumVar(0, solver.infinity(), V[i])
    v[5] = solver.NumVar(0, solver.infinity(), V[5])
    D0 = solver.NumVar(0.4, 0.4, 'D0')
    d = [0.1, 0.2, 0.3]
    de = [0.1, 0.05, 0.15]

    constraint = [
        [0, solver.infinity()],
        [0, solver.infinity()],
        [0, solver.infinity()],
        [-solver.infinity(), 1],
        [-solver.infinity(), 1],
        [-solver.infinity(), 1],
        [-solver.infinity(), 0],  # 0.4*(1-d[2])*de[2]/(1-d[2]*de[2])],
        [-solver.infinity(), 0],
        [-solver.infinity(), 0]
    ]
    coef = [
        [
            0,                              #k1
            0,                              #k2
            1,                              #k3
            (1-d[2])*de[2]/(1-d[2]*de[2]),  #e
            -(1-de[2])/(1-d[2]*de[2]),      #R1
            -(1-de[2])/(1-d[2]*de[2])       #R2
        ],
        [
            1,                              #k1
            0,                              #k2
            0,                              #k3
            0,                              #e
            -(1-de[0])/(1-d[0]*de[0]),      #R1
            0                               #R2
        ],
        [
            0,                              #k1
            1,                              #k2
            0,                              #k3
            0,                              #e
            0,                              #R1
            -(1-de[1])/(1-d[1]*de[1]),      #R2
        ],
        [
            0,                              #k1
            0,                              #k2
            1/((1-d[2])*de[2]),             #k3
            1/(1-d[2]),                     #e
            1/(1-d[2]),                     #R1
            1/(1-d[2]),                     #R2
        ],
        [
            1/((1-d[0])*de[0]),             #k1
            0,                              #k2
            0,                              #k3
            0,                              #e
            1/(1-d[0]),                     #R1
            0,                              #R2
        ],
        [
            0,                              #k1
            1/((1-d[1])*de[1]),             #k2
            0,                              #k3
            0,                              #e
            0,                              #R1
            1/(1-d[1])                      #R2
        ],
       [
           0,                              #k1
           0,                              #k2
           1,                              #k3
           (1-d[2])*de[2]/(1-d[2]*de[2]),  #e
           0,                              #R1
           0                               #R2
       ],
        [
            1,                              #k1
            0,                              #k2
            -(1-d[0])*de[0]/((1-d[0]*de[0])*de[2]),                              #k3
            -(1-d[0])*de[0]/(1-d[0]*de[0]),  #e
            0,                              #R1
            0                               #R2
        ],
        [
            0,                              #k1
            1,                              #k2
            -(1-d[1])*de[1]/((1-d[1]*de[1])*de[2]),                             #k3
            -(1-d[1])*de[1]/(1-d[1]*de[1]),  #e
            0,                              #R1
            0                               #R2
        ],
    ]
    c = [None] * 9 
    for i in range(9):
        c[i] = solver.Constraint(constraint[i][0], constraint[i][1])
        for j in range(6):
            c[i].SetCoefficient(v[j], coef[i][j])
    c[6].SetCoefficient(D0, -(1-d[2])*de[2]/(1-d[2]*de[2]))

    objective = solver.Objective()
    objective.SetCoefficient(v[4], v1)
    objective.SetCoefficient(v[5], v2)
    objective.SetMaximization()

    solver.Solve()
    opt_solution = v[4].solution_value() + v[5].solution_value()
    print('Optimal objective value =', opt_solution)
    return [v[4].solution_value(), v[5].solution_value()]

if __name__ == '__main__':
    print(RYNetwork(1, 1))
