from __future__ import print_function
from ortools.linear_solver import pywraplp

def YNetwork(v1, v2):
  # Instantiate a Glop solver, naming it Y-Netowrk.
  solver = pywraplp.Solver('Y-Network',
                           pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

# Create the two variables and let them take on any value.
  R1 = solver.NumVar(0, solver.infinity(), 'R1')
  # R1 = solver.NumVar(0, solver., 'R1')
  R2 = solver.NumVar(0, solver.infinity(), 'R2')
  k1 = solver.NumVar(0, solver.infinity(), 'k1')
  k2 = solver.NumVar(0, solver.infinity(), 'k2')
  k3 = solver.NumVar(0, solver.infinity(), 'k3')
  RP1 = solver.NumVar(0, solver.infinity(), 'RP1')
  RP2 = solver.NumVar(0, solver.infinity(), 'RP2')
  val = [R1, R2, k1, k2, k3, RP1, RP2]
  d1, de1 = 0.2, 0.05
  d2, de2 = 0.3, 0.05
  d3, de3 = 0.25, 0.05

  c1 = solver.Constraint(0, solver.infinity(), 'c1')
  c1.SetCoefficient(k1, (1-d1*de1))
  c1.SetCoefficient(R1, -(1-de1))

  c2 = solver.Constraint(0, solver.infinity())
  c2.SetCoefficient(k2, (1-d2*de2))
  c2.SetCoefficient(R2, -(1-de2))

  c3 = solver.Constraint(0, solver.infinity())
  c3.SetCoefficient(k3, (1-d3*de3))
  c3.SetCoefficient(R1, -(1-de3))
  c3.SetCoefficient(R2, -(1-de3))

  c4 = solver.Constraint(-solver.infinity(), (1-d1))
  c4.SetCoefficient(k1, 1/(de1))
  c4.SetCoefficient(R1, 1)

  c5 = solver.Constraint(-solver.infinity(), (1-d2))
  c5.SetCoefficient(k2, 1/(de2))
  c5.SetCoefficient(R2, 1)

  c6 = solver.Constraint(-solver.infinity(), (1-d3))
  c6.SetCoefficient(k3, 1/(de3))
  c6.SetCoefficient(R2, 1)
  c6.SetCoefficient(R1, 1)

  c7 = solver.Constraint(-solver.infinity(), 0)
  c7.SetCoefficient(k3, (1-d3*de3))
  c7.SetCoefficient(k2, -(1-d3)*de3/(de2))
  c7.SetCoefficient(k1, -(1-d3)*de3/(de1))
  
  c8 = solver.Constraint(-solver.infinity(), 0)
  c8.SetCoefficient(k1, (1-d1*de1))
  c8.SetCoefficient(RP1, -(1-d1)*de1)

  c9 = solver.Constraint(-solver.infinity(), 0)
  c9.SetCoefficient(k2, (1-d2*de2))
  c9.SetCoefficient(RP2, -(1-d2)*de2)

  objective = solver.Objective()
  objective.SetCoefficient(R1, v1)
  objective.SetCoefficient(R2, v2)
  objective.SetMaximization()


  # print(solver.RowConstraint().GetCoefficient(k1))
  # print(solver.LookupConstraint('c1').GetCoefficient(k1))
  # print(dir(c9.this))
  # print(R1.name())
  # Solve the system.
  solver.Solve()
  # print(dir(c9))
  opt_solution = R1.solution_value() + R2.solution_value()

  # for i in val:
  #   print(i.name() + " = " + str(i.solution_value()))
  # print('Number of variables =', solver.NumVariables())
  # print('Number of constraints =', solver.NumConstraints())
  # The value of each variable in the solution.
  # print('Solution:')
  # print('R1 = ', R1.solution_value())
  # print('R2 = ', R2.solution_value())
  # The objective value of the solution.
  print('Optimal objective value =', opt_solution)
  return R1.solution_value(), R2.solution_value()

if __name__ == '__main__':
  print(YNetwork(1, 10))
  # print(YNetwork(1, 2))
