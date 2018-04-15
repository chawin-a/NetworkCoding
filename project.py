from __future__ import print_function
from ortools.linear_solver import pywraplp
from pprint import pprint
from sys import argv

def show(num):
    return "{:.3f}".format(num)

def DEBUG(text):
    if len(argv) > 1:
        if argv[1] == 'debug':
            print(text)

class SolverLP:

    solver = pywraplp.Solver('Network', pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)
    
    # Constraints
    constraints = dict()

    # Variables
    R = dict() # Messages in nodes
    k = dict() # Random bits k (on edges)
    e = dict() # Random bits e (on edges)
    E = dict() # Random bits in nodes
    f_k = dict() # Flow of Keys
    f_R = dict() # Flow of Messages

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
        DEBUG("==== Constraints on Edges ====")
        DEBUG('--Key--')
        for u, v, d, r in self.edges:
            edge = (u, v)
            # Constraint 1 Random bits for generate Key
            mp = ('Key', edge)
            self.constraints[mp] = self.solver.Constraint(0, INF)
            self.constraints[mp].SetCoefficient(self.k[edge], 1-d*r)
            self.constraints[mp].SetCoefficient(self.e[edge], (1-d)*r)
            self.constraints[mp].SetCoefficient(self.R[edge], -(1-r))
            DEBUG(show(1-d*r) + ' ' + self.k[edge].name() + ' + ' +
                  show((1-d)*r) + ' ' + self.e[edge].name() + ' >= ' +
                  show((1-r)) + ' ' + self.R[edge].name() )
        DEBUG('--Capacity--')
        for u, v, d, r in self.edges:
            edge = (u, v)
            # Constraint 2 Capacity
            mp = ('Cap', edge)
            self.constraints[mp] = self.solver.Constraint(-INF, 1-d)
            self.constraints[mp].SetCoefficient(self.R[edge], 1)
            self.constraints[mp].SetCoefficient(self.k[edge], 1/r)
            self.constraints[mp].SetCoefficient(self.e[edge], 1)
            # print('--Capacity--')
            DEBUG(self.R[edge].name() + ' + ' +
                  show(1/r) + ' ' + self.k[edge].name() + ' + ' +
                  self.e[edge].name() + ' <= ' + show(1-d))
        DEBUG('--Limit of random bits--')
        for u, v, d, r in self.edges:
            edge = (u, v)
            # Constraint 3 u has random bits for sending
            mp = ('LM', edge)
            self.constraints[mp] = self.solver.Constraint(-INF, 0)
            self.constraints[mp].SetCoefficient(self.k[edge], 1-d*r)
            self.constraints[mp].SetCoefficient(self.E[u], -(1-d)*r)
            self.constraints[mp].SetCoefficient(self.e[edge], (1-d)*r)
            DEBUG(show(1-d*r) + ' ' + self.k[edge].name() + ' <= ' +
                  show((1-d)*r) + ' ' + '(' + self.E[u].name() + ' - ' + self.e[edge].name() + ')')
        DEBUG('--Summation of e from source--')
        for u, v, d, r in self.edges:
            edge = (u, v)
            # Constraint 4 Sum e_e = Sum e_e from Si
            mp = ('Sum_e_e', edge)
            self.constraints[mp] = self.solver.Constraint(0, 0)
            self.constraints[mp].SetCoefficient(self.e[edge], 1)
            plus = False
            out = self.e[edge].name() + ' = '
            for w in self.source:
                s_e = (w, 'to', edge)
                nm = 'e_('+str(w)+' to '+str(edge)+')'
                self.e[s_e] = self.solver.NumVar(0, INF, nm)
                self.constraints[mp].SetCoefficient(self.e[s_e], -1)
                out += (' + ' if plus else '') + self.e[s_e].name()
                plus = True
            DEBUG(out)
        DEBUG('--Summation of k from source--')
        for u, v, d, r in self.edges:
            edge = (u, v)
            # Constraint 5 Sum ke = Sum ke from Si
            mp = ('Sum_k_e', edge)
            self.constraints[mp] = self.solver.Constraint(0, 0)
            self.constraints[mp].SetCoefficient(self.k[edge], 1)
            plus = False
            out = self.k[edge].name() + ' = '
            for w in self.source:
                s_e = (w, 'to', edge)
                nm = 'k_('+str(w)+' to '+str(edge)+')'
                self.k[s_e] = self.solver.NumVar(0, INF, nm)
                self.constraints[mp].SetCoefficient(self.k[s_e], -1)
                out += (' + ' if plus else '') + self.k[s_e].name()
                plus = True
            DEBUG(out)
        DEBUG('--Summation of E from source--')
        for u in range(self.nodes):
            if u not in self.source:
                # Constraint 6 Eu = Sum Eu from Si
                mp = ('Sum_E_u', u)
                self.constraints[mp] = self.solver.Constraint(0, 0)
                self.constraints[mp].SetCoefficient(self.E[u], 1)
                out = self.E[u].name() + ' = '
                plus = False
                for s in self.source:
                    s_u = (s, 'to', u)
                    nm = 'E_('+str(s)+' to '+str(u)+')'
                    self.E[s_u] = self.solver.NumVar(0, INF, nm)
                    self.constraints[mp].SetCoefficient(self.E[s_u], -1)
                    out += (' + ' if plus else '') + self.E[s_u].name()
                    plus = True
                DEBUG(out)
        DEBUG('--Summation of R from source--')
        for u in range(self.nodes):
            if u not in self.source:
                mp = ('Sum_R_u', u)
                self.constraints[mp] = self.solver.Constraint(0, 0)
                self.constraints[mp].SetCoefficient(self.R[u], 1)
                out = self.R[u].name() + ' = '
                plus = False
                for s in self.source:
                    s_u = (s, 'to', u)
                    nm = 'R_('+str(s)+' to '+str(u)+')'
                    self.R[s_u] = self.solver.NumVar(0, INF, nm)
                    self.constraints[mp].SetCoefficient(self.R[s_u], -1)
                    out += (' + ' if plus else '') + self.R[s_u].name()
                    plus = True
                DEBUG(out)

    def createFlowConstraint(self):
        INF = self.solver.infinity()
        DEBUG("==== Flow Constraints ====")
        # Flow in == Flow out
        DEBUG('--Flow in == Flow out--')
        for s in self.source:
            for d in self.destination:
                for n in range(self.nodes):
                     # s to d through n
                    if n != s and n != d:
                        flow = ('Flow_in_out', s, d, n)
                        self.constraints[flow] = self.solver.Constraint(0, 0)
                        out = ''
                        plus = False
                        # IN
                        if n in self.reverse_graph:
                            for u, de, re, i in self.reverse_graph[n]:
                                edge = (u, n)
                                key = (s, d, edge)
                                nm = 'f_k('+str(s)+' to '+str(d)+','+str(edge)+')'
                                if key not in self.f_k:
                                    self.f_k[key] = self.solver.NumVar(0, INF, nm)
                                self.constraints[flow].SetCoefficient(self.f_k[key], 1)
                                out += (' + ' if plus else '') + self.f_k[key].name()
                                plus = True
                        # OUT
                        if n in self.graph:
                            for v, de, re, i in self.graph[n]:
                                edge = (n, v)
                                key = (s, d, edge)
                                nm = 'f_k('+str(s)+' to '+str(d)+','+str(edge)+')'
                                if key not in self.f_k:
                                    self.f_k[key] = self.solver.NumVar(0, INF, nm)
                                self.constraints[flow].SetCoefficient(self.f_k[key], -1)
                                out += ' - ' + self.f_k[key].name()
                        out += ' = 0'
                        DEBUG(out)
        # Flow Capacity
        DEBUG('--Flow capacity--')
        for s in self.source:
            for u, v, de, re in self.edges:
                edge = (u, v)
                mp = ('FlowCap', s, edge)
                self.constraints[mp] = self.solver.Constraint(-INF, 0)
                out = ''
                plus = False
                for d in self.destination:
                    key = (s, d, edge)
                    self.constraints[mp].SetCoefficient(self.f_k[key], 1)
                    out += (' + ' if plus else '') + self.f_k[key].name()
                    plus = True
                s_e = (s, 'to', edge)
                self.constraints[mp].SetCoefficient(self.k[s_e], -1/re)
                self.constraints[mp].SetCoefficient(self.e[s_e], -1)
                out += ' <= ' + show(1/re) + ' ' + self.k[s_e].name() + ' + ' + self.e[s_e].name()
                DEBUG(out)
        # Limit of Flow
        DEBUG('--Limit flow--')
        for s in self.source:
            mp = ('LM_Flow', s)
            self.constraints[mp] = self.solver.Constraint(-INF, 0)
            self.constraints[mp].SetCoefficient(self.E[s], -1)
            out = ''
            plus = False
            for d in self.destination:
                for v, de, re, i in self.graph[s]:
                    edge = (s, v)
                    key = (s, d, edge)
                    self.constraints[mp].SetCoefficient(self.f_k[key], 1)
                    out += (' + ' if plus else '') + self.f_k[key].name()
                    plus = True
            out += ' <= ' + self.E[s].name()
            DEBUG(out)
        
        # Limit of E from Source i
        DEBUG('--Limit random bits--')
        for u in range(self.nodes):
            if u not in self.source:
                for s in self.source:
                    s_u = (s, 'to', u)
                    mp = ('LM_bits', s_u)
                    self.constraints[mp] = self.solver.Constraint(-INF, 0)
                    self.constraints[mp].SetCoefficient(self.E[s_u], 1)
                    out = self.E[s_u].name() + ' <= '
                    plus = False
                    for d in self.destination:
                        for v, de, re, i in self.reverse_graph[u]:
                            edge = (v, u)
                            key = (s, d, edge)
                            self.constraints[mp].SetCoefficient(self.f_k[key], -1)
                            out += (' + ' if plus else '') + self.f_k[key].name()
                            plus = True
                    DEBUG(out)

    def createConstraints(self):
        self.createSecurityConstraint()
        self.createFlowConstraint()
    
    def Solve(self):
        self.createVariables()
        self.createConstraints()
        self.objective = self.solver.Objective()
        DEBUG(self.solver.NumConstraints())


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
