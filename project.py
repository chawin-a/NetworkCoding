from __future__ import print_function
from ortools.linear_solver import pywraplp

class SolverLP:

    solver = pywraplp.Solver('Network', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)
    constraints = dict()
    R = dict() # Messages in nodes
    k = dict() # Random bits k (on edges)
    e = dict() # Random bits e (on edges)
    E = dict() # Random bits in nodes

    def __init__(self, nodes, num_edges, edges, source, destination, graph, reverse_graph, s_to_d):
        self.nodes = nodes
        self.num_edges = num_edges
        self.edges = edges
        self.source = source
        self.destination = destination
        self.graph = graph
        self.reverse_graph = reverse_graph
        self.s_to_d = s_to_d

    def createVariables(self):
        INF = self.solver.infinity()
        # Variables in nodes
        for i in range(self.nodes):
            self.R[i] = self.solver.NumVar(0, INF, 'R_'+str(i))
            self.E[i] = self.solver.NumVar(0, INF, 'E_'+str(i))
        
        # Variables on edges
        for u, v, d, r in self.edges:
            edge = (u, v)
            nm = 'k_('+str(u)+','+str(v)+')'
            self.k[edge] = self.solver.NumVar(0, INF, nm)
            nm = 'e_('+str(u)+','+str(v)+')'
            self.e[edge] = self.solver.NumVar(0, INF, nm)
            nm = 'R_('+str(u)+','+str(v)+')'
            self.R[edge] = self.solver.NumVar(0, INF, nm)

    def createSecurityConstraint(self):
        INF = self.solver.infinity()
        for u, v, d, r in self.edges:
            edge = (u, v)
            # Constraint 1 Random bits for generate Key
            mp = ('Key', edge)
            self.constraints[mp] = self.solver.Constraint(0, INF)
            self.constraints[mp].SetCoefficient(self.k[edge], 1-d*r)
            self.constraints[mp].SetCoefficient(self.e[edge], (1-d)*r)
            self.constraints[mp].SetCoefficient(self.R[edge], -(1-r))

            # Constraint 2 Capacity
            mp = ('Cap', edge)
            self.constraints[mp] = self.solver.Constraint(-INF, 1-d)
            self.constraints[mp].SetCoefficient(self.R[edge], 1)
            self.constraints[mp].SetCoefficient(self.k[edge], 1/r)
            self.constraints[mp].SetCoefficient(self.e[edge], 1)

            # Constraint 3 u has random bits for sending
            mp = ('LM', edge)
            self.constraints[mp] = self.solver.Constraint(-INF, 0)
            self.constraints[mp].SetCoefficient(self.k[edge], 1-d*r)
            self.constraints[mp].SetCoefficient(self.E[u], -(1-d)*r)
            self.constraints[mp].SetCoefficient(self.e[edge], (1-d)*r)

            # Constraint 4 Sum e_e = Sum e_e from Si
            mp = ('Sum_e_e', edge)
            self.constraints[mp] = self.solver.Constraint(0, 0)
            self.constraints[mp].SetCoefficient(self.e[edge], 1)
            for w in self.source:
                s_e = (w, 'to', edge)
                nm = 'E_('+str(w)+' to ('+str(edge)+'))'
                self.e[s_e] = self.solver.NumVar(0, INF, nm)
                self.constraints[mp].SetCoefficient(self.e[s_e], -1)

            # Constraint 5 Sum ke = Sum ke from Si
            mp = ('Sum_k_e', edge)
            self.constraints[mp] = self.solver.Constraint(0, 0)
            self.constraints[mp].SetCoefficient(self.k[edge], 1)
            for w in self.source:
                s_e = (w, 'to', edge)
                nm = 'k_('+str(w)+' to ('+str(edge)+'))'
                self.k[s_e] = self.solver.NumVar(0, INF, nm)
                self.constraints[mp].SetCoefficient(self.k[s_e], -1)

        for u in range(self.nodes):
            if u not in self.source:
                # Constraint 6 Eu = Sum Eu from Si
                mp = ('Sum_E_u', u)
                self.constraints[mp] = self.solver.Constraint(0, 0)
                self.constraints[mp].SetCoefficient(self.E[u], 1)
                for s in self.source:
                    s_u = (s, 'to', u)
                    nm = 'E_('+str(s)+' to '+str(u)+')'
                    self.E[s_u] = self.solver.NumVar(0, INF, nm)
                    self.constraints[mp].SetCoefficient(self.E[s_u], -1)

    def createFlowConstraint(self):
        INF = self.solver.infinity()

    def createConstraints(self):
        self.createSecurityConstraint()
        self.createFlowConstraint()
    
    def Solve(self):
        self.createVariables()
        self.createConstraints()
        self.objective = self.solver.Objective()
        print(self.solver.NumConstraints())


def main():
    g = dict()
    rg = dict()
    source = []
    destination = []
    source_len = 0
    des_len = 0
    nodes = int(input())
    edges = int(input())
    edge = []
    s_to_d = []
    for i in range(edges):
        inp = input().split()
        u, v, d, r = int(inp[0]), int(inp[1]), float(inp[2]), float(inp[3])
        edge.append((u, v, d, r))
        if u in g:
            g[u].append((v, d, r, i))
        else:
            g[u] = [(v, d, r, i)]
        if v in rg:
            rg[v].append((u, d, r, i))
        else:
            rg[v] = [(u, d, r, i)]

    source_len = int(input())
    for i in range(source_len):
        source.append(int(input()))
    des_len = int(input())
    for i in range(des_len):
        destination.append(int(input()))
    for i in range(int(input())):
        s_to_d.append((int(input()), int(input())))
    solv = SolverLP(nodes, edges, edge, source, destination, g, rg, s_to_d)
    solv.Solve()

if __name__ == '__main__':
    main()
