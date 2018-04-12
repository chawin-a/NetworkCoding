from __future__ import print_function
from ortools.linear_solver import pywraplp


def createLP():
    nodes = 0
    edges = 0
    g = dict()
    rg = dict()
    adj = dict()
    source = []
    destination = []
    source_len = 0
    des_len = 0
    print("Number of nodes: ", end="")
    nodes = int(input())
    print("Number of edges: ", end="")
    edges = int(input())
    for i in range(edges):
        print("#%d (u, v, d, r) = " % (i+1), end="")
        inp = input().split()
        u, v, d, r = int(inp[0]), int(inp[1]), float(inp[2]), float(inp[3])
        if u in g:
            g[u].append((v, d, r, i))
        else:
            g[u] = [(v, d, r, i)]
        if v in rg:
            rg[v].append((u, d, r, i))
        else:
            rg[v] = [(u, d, r, i)]

    print("Number of source: ", end="")
    source_len = int(input())
    for i in range(source_len):
        source.append(int(input()))
    print("Number of destination: ", end="")
    des_len = int(input())
    for i in range(des_len):
        destination.append(int(input()))
    
    solver = pywraplp.Solver('Network', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

    queue = []
    K = [None] * edges
    R = [None] * edges
    print(R)
    for i in range(edges):
        K[i] = solver.NumVar(0, solver.infinity(), 'K'+str(i+1))
        R[i] = solver.NumVar(0, solver.infinity(), 'R'+str(i+1))
        
    for i in range(source_len):
        queue.append(source[i])

    visited = [False] * nodes

    constraints = dict()

    while len(queue) != 0:
        u = queue[0]
        queue.pop(0)
        if visited[u] == True:
            continue
        visited[u] = True
        if u in destination:
            continue
        for v, d, r, i in g[u]:
            print(v, d, r, i)
            constraints[(u, v)] = dict()
            constraints[(u, v)]['Key'] = solver.Constraint(0, solver.infinity())
            constraints[(u, v)]['Key'].SetCoefficient(K[i], 1)
            constraints[(u, v)]['Key'].SetCoefficient(R[i], -(1-r)/(1-d*r))
            constraints[(u, v)]['Cap'] = solver.Constraint(-solver.infinity(), 1)
            constraints[(u, v)]['Cap'].SetCoefficient(K[i], 1/((1-d)*r))
            constraints[(u, v)]['Cap'].SetCoefficient(R[i], 1/(1-d))
            if v not in destination:
                for vv, dd, rr, ii in g[v]:
                    if vv in constraints:
                        constraints[vv].SetCoefficient(K[i], (-1/r)*((1-dd)*rr/(1-dd*rr)))
                    else:
                        constraints[vv] = solver.Constraint(-solver.infinity(), 0)
                        constraints[vv].SetCoefficient(K[ii], 1)
                        constraints[vv].SetCoefficient(K[i], (-1/r)*((1-dd)*rr/(1-dd*rr)))
            queue.append(v)
    
    constraints['Data'] = solver.Constraint(-solver.infinity(), 0)
    for u in source:
        for v, d, r, i in g[u]:
            constraints['Data'].SetCoefficient(R[i], -1)
    for u in destination:
        for v, d, r, i in rg[u]:
            constraints['Data'].SetCoefficient(R[i], 1)

    objective = solver.Objective()
    for u in destination:
        for v, d, r, i in rg[u]:
            objective.SetCoefficient(R[i], 1)
    objective.SetMaximization()

    solver.Solve()
    opt_solution = 0
    for u in destination:
        for v, d, r, i in rg[u]:
            opt_solution += R[i].solution_value()
    
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    print('Optimal objective value =', opt_solution)

if __name__ == '__main__':
    createLP()
