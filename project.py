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

    def __init__(self, nodes, num_edges, edges, source, destination, graph, reverse_graph, s_to_d, lim_s):
        self.nodes = nodes
        self.num_edges = num_edges
        self.edges = edges
        self.source = source
        self.destination = destination
        self.graph = graph
        self.reverse_graph = reverse_graph
        self.s_to_d = s_to_d
        self.lim_s = lim_s
        self.createVariables()
        self.createConstraints()

    def createVariables(self):
        INF = self.solver.infinity()
        # Variables in nodes
        for i in range(self.nodes):
            self.R[i] = self.solver.NumVar(0, INF, 'R_'+str(i))
            if i in self.lim_s:
                self.E[i] = self.solver.NumVar(self.lim_s[i], self.lim_s[i], 'E_'+str(i))
            else:
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
        
        # Create Flow Variables
        for s in self.source:
            for d in range(self.nodes):
                if d in self.source:
                    continue
                for u, v, de, re in self.edges:
                    edge = (u, v)
                    key = (s, d, edge)
                    nm = 'f_k('+str(s)+' to '+str(d)+','+str(edge)+')'
                    self.f_k[key] = self.solver.NumVar(0, INF, nm)
        
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
        DEBUG('--Summation of R_u from source--')
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
        DEBUG('--Summation of R_e from source--')
        for u, v, d, r in self.edges:
            edge = (u, v)
            mp = ('Sum_R_e', edge)
            self.constraints[mp] = self.solver.Constraint(0, 0)
            self.constraints[mp].SetCoefficient(self.R[edge], 1)
            plus = False
            out = self.R[edge].name() + ' = '
            for w in self.source:
                s_e = (w, 'to', edge)
                nm = 'R_('+str(w)+' to '+str(edge)+')'
                self.R[s_e] = self.solver.NumVar(0, INF, nm)
                self.constraints[mp].SetCoefficient(self.R[s_e], -1)
                out += (' + ' if plus else '') + self.R[s_e].name()
                plus = True
            DEBUG(out)

    def createFlowConstraint(self):
        INF = self.solver.infinity()

        DEBUG("==== Flow Constraints ====")
        # Flow in == Flow out
        DEBUG('--Flow in == Flow out--')
        for s in self.source:
            for d in range(self.nodes):
                if d in self.source:
                    continue
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
                                self.constraints[flow].SetCoefficient(self.f_k[key], 1)
                                out += (' + ' if plus else '') + self.f_k[key].name()
                                plus = True
                        # OUT
                        if n in self.graph:
                            for v, de, re, i in self.graph[n]:
                                edge = (n, v)
                                key = (s, d, edge)
                                nm = 'f_k('+str(s)+' to '+str(d)+','+str(edge)+')'
                                self.constraints[flow].SetCoefficient(self.f_k[key], -1)
                                out += ' - ' + self.f_k[key].name()
                        out += ' = 0'
                        DEBUG(out)
        # Flow Capacity
        DEBUG('--Flow capacity--')
        for s in self.source:
            for u, v, de, re in self.edges:
                edge = (u, v)
                for d in range(self.nodes):
                    if d in self.source:
                        continue
                    mp = ('FlowCap', s, d, edge)
                    self.constraints[mp] = self.solver.Constraint(-INF, 0)
                    key = (s, d, edge)
                    self.constraints[mp].SetCoefficient(self.f_k[key], 1)
                    out = self.f_k[key].name()
                    s_e = (s, 'to', edge)
                    self.constraints[mp].SetCoefficient(self.k[s_e], -1/re)
                    self.constraints[mp].SetCoefficient(self.e[s_e], -1)
                    out += ' <= ' + show(1/re) + ' ' + self.k[s_e].name() + ' + ' + self.e[s_e].name()
                    DEBUG(out)
        # Limit of Flow
        DEBUG('--Limit flow--')
        for s in self.source:
            for d in range(self.nodes):
                if d in self.source:
                    continue
                mp = ('LM_Flow', s, d)
                self.constraints[mp] = self.solver.Constraint(-INF, 0)
                self.constraints[mp].SetCoefficient(self.E[s], -1)
                out = ''
                plus = False
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
                    for v, de, re, i in self.reverse_graph[u]:
                        edge = (v, u)
                        key = (s, u, edge)
                        self.constraints[mp].SetCoefficient(self.f_k[key], -1)
                        out += (' + ' if plus else '') + self.f_k[key].name()
                        plus = True
                    DEBUG(out)
        
        # Flow_R in == Flow_R out
        DEBUG('--Flow_R in == Flow_R out--')
        for s, d in self.s_to_d:
            for n in range(self.nodes):
                    # s to d through n
                if n != s and n != d:
                    flow = ('Flow_R_in_out', s, d, n)
                    self.constraints[flow] = self.solver.Constraint(0, 0)
                    out = ''
                    plus = False
                    # IN
                    if n in self.reverse_graph:
                        for u, de, re, i in self.reverse_graph[n]:
                            edge = (u, n)
                            key = (s, d, edge)
                            nm = 'f_R('+str(s)+' to '+str(d)+','+str(edge)+')'
                            if key not in self.f_R:
                                self.f_R[key] = self.solver.NumVar(0, INF, nm)
                            self.constraints[flow].SetCoefficient(self.f_R[key], 1)
                            out += (' + ' if plus else '') + self.f_R[key].name()
                            plus = True
                    # OUT
                    if n in self.graph:
                        for v, de, re, i in self.graph[n]:
                            edge = (n, v)
                            key = (s, d, edge)
                            nm = 'f_R('+str(s)+' to '+str(d)+','+str(edge)+')'
                            if key not in self.f_R:
                                self.f_R[key] = self.solver.NumVar(0, INF, nm)
                            self.constraints[flow].SetCoefficient(self.f_R[key], -1)
                            out += ' - ' + self.f_R[key].name()
                    out += ' = 0'
                    DEBUG(out)

        # Flow_R Capacity
        DEBUG('--Flow_R capacity--')
        to = dict()
        for s, d in self.s_to_d:
            if s not in to:
                to[s] = [d]
            else:
                to[s].append(d)
        for s in self.source:
            for u, v, de, re in self.edges:
                edge = (u, v)
                mp = ('Flow_R_Cap', s, edge)
                self.constraints[mp] = self.solver.Constraint(-INF, 0)
                out = ''
                plus = False
                for d in to[s]:
                    key = (s, d, edge)
                    self.constraints[mp].SetCoefficient(self.f_R[key], 1)
                    out += (' + ' if plus else '') + self.f_R[key].name()
                    plus = True
                s_e = (s, 'to', edge)
                self.constraints[mp].SetCoefficient(self.R[s_e], -1)
                out += ' <= ' + self.R[s_e].name()
                DEBUG(out)
        
        # Limit_R of Flow
        DEBUG('--Limit flow_R--')
        for s in self.source:
            mp = ('LM_Flow_R', s)
            self.constraints[mp] = self.solver.Constraint(-INF, 0)
            self.constraints[mp].SetCoefficient(self.R[s], -1)
            out = ''
            plus = False
            for d in to[s]:
                for v, de, re, i in self.graph[s]:
                    edge = (s, v)
                    key = (s, d, edge)
                    self.constraints[mp].SetCoefficient(self.f_R[key], 1)
                    out += (' + ' if plus else '') + self.f_R[key].name()
                    plus = True
            out += ' <= ' + self.R[s].name()
            DEBUG(out)
        
        # Limit of R from Source i
        DEBUG('--Limit messages--')
        for u in self.destination:
            if u not in self.source:
                for s in self.source:
                    s_u = (s, 'to', u)
                    mp = ('LM_R_bits', s_u)
                    self.constraints[mp] = self.solver.Constraint(-INF, 0)
                    self.constraints[mp].SetCoefficient(self.R[s_u], 1)
                    out = self.R[s_u].name() + ' <= '
                    plus = False
                    for d in to[s]:
                        for v, de, re, i in self.reverse_graph[u]:
                            edge = (v, u)
                            key = (s, d, edge)
                            self.constraints[mp].SetCoefficient(self.f_R[key], -1)
                            out += (' + ' if plus else '') + self.f_R[key].name()
                            plus = True
                    DEBUG(out)

    def createConstraints(self):
        self.createSecurityConstraint()
        self.createFlowConstraint()
    
    def createObjective(self):
        self.objective = self.solver.Objective()
        for u in self.destination:
            self.objective.SetCoefficient(self.R[u], 1)
        self.objective.SetMaximization()

    def resultValue(self):
        if len(argv) > 1:
            if argv[1] == 'debug':
                opt_sol = 0
                for u in self.destination:
                    opt_sol += self.R[u].solution_value()
                print('opt_solution =', opt_sol)
                for var in [self.R, self.E, self.k, self.e, self.f_k, self.f_R]:
                    for k in var.keys():
                        print(var[k].name() + ' = ' + str(var[k].solution_value()))
        else:
            opt_sol = 0
            for u in self.destination:
                opt_sol += self.R[u].solution_value()
            print('opt_solution =', opt_sol)

    def Solve(self):
        DEBUG(self.solver.NumConstraints())
        self.createObjective()
        self.solver.Solve()
        self.resultValue()

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
        (s, d) = [int(x) for x in input().split()]
        s_to_d.append((s, d))
    lim_s = dict()
    for i in range(int(input())):
        inp = input().split()
        lim_s[int(inp[0])] = float(inp[1])

    solv = SolverLP(nodes, edges, edge, source, destination, g, rg, s_to_d, lim_s)
    solv.Solve()

if __name__ == '__main__':
    main()
