from __future__ import print_function
from ortools.linear_solver import pywraplp

def XNetwork():
  # Instantiate a Glop solver, naming it Y-Netowrk.
  solver = pywraplp.Solver('X-Network',
                           pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

# Create the two variables and let them take on any value.
  R = [None] * 2
  for i in range(2):
      R[i] = solver.NumVar(0, solver.infinity(), 'R'+str(i+1))
  K = [None] * 5
  for i in range(5):
      K[i] = solver.NumVar(0, solver.infinity(), 'K'+str(i+1))
  d = [0.1, 0.2, 0.3, 0.4, 0.5]
  r = [0.1, 0.05, 0.15, 0.35, 0.2]
#  e = solver.NumVar(0, solver.infinity(), 'e')
  
  c1 = [None] * 2
  for i in range(2):
    c1[i] = solver.Constraint(0, solver.infinity())
    c1[i].SetCoefficient(K[i], 1)
    c1[i].SetCoefficient(R[i], -(1-r[i])/(1-d[i]*r[i]))
  
  c2 = solver.Constraint(0, solver.infinity())
  c2.SetCoefficient(K[2], 1)
  #c2.SetCoefficient(e, (1-d[2])*r[2]/(1-d[2]*r[2]))
  for i in range(2):
      c2.SetCoefficient(R[i], -(1-r[2])/(1-d[2]*r[2]))

  c3 = [None] * 2
  for i in range(3, 5):
      c3[i-3] = solver.Constraint(0, solver.infinity())
      c3[i-3].SetCoefficient(K[i], 1)
      c3[i-3].SetCoefficient(R[i-3], -(1-r[i])/(1-d[i]*r[i])) 

  c4 = [None] * 2
  for i in range(2):
      c4[i] = solver.Constraint(-solver.infinity(), 1)
      c4[i].SetCoefficient(R[i], 1/(1-d[i]))
      c4[i].SetCoefficient(K[i], 1/((1-d[i])*r[i]))
  c5 = [None] * 2
  for i in range(3, 5):
      c5[i-3] = solver.Constraint(-solver.infinity(), 1)
      c5[i-3].SetCoefficient(R[i-3], 1/(1-d[i]))
      c5[i-3].SetCoefficient(K[i], 1/((1-d[i])*r[i]))
  
  c6 = solver.Constraint(-solver.infinity(), 1)
  for i in range(2):
      c6.SetCoefficient(R[i], 1/(1-d[2]))
  c6.SetCoefficient(K[2], 1/((1-d[2])*r[2]))
#  c6.SetCoefficient(e, 1/(1-d[2]))

  c7 = solver.Constraint(-solver.infinity(), 0)
  c7.SetCoefficient(K[2], 1)
  for i in range(2):
      c7.SetCoefficient(K[i], -(1-d[2])*r[2]/(r[i]*(1-d[2]*r[2])))
#  c7.SetCoefficient(e, (1-d[2])*r[2]/(1-d[2]*r[2]) )
  
  c8 = [None] * 2
  for i in range(3, 5):
      c8[i-3] = solver.Constraint(-solver.infinity(), 0)
      c8[i-3].SetCoefficient(K[i], 1)
 #     c8[i-3].SetCoefficient(e, -(1-d[i])*r[i]/(1-d[i]*r[i]))
      c8[i-3].SetCoefficient(K[2], -(1-d[i])*r[i]/(r[2]*(1-d[i]*r[i])) )

  objective = solver.Objective()
  objective.SetCoefficient(R[0], 1)
  objective.SetCoefficient(R[1], 1)
  objective.SetMaximization()

  # Solve the system.
  solver.Solve()
  opt_solution = R[0].solution_value() + R[1].solution_value()
  print('Optimal objective value =', opt_solution)
  # print('Number of variables =', solver.NumVariables())
  # print('Number of constraints =', solver.NumConstraints())
  # The value of each variable in the solution.
  # print('Solution:')
  # print('R1 = ', R1.solution_value())
  # print('R2 = ', R2.solution_value())
  # The objective value of the solution.
  # print('Optimal objective value =', opt_solution)
  return [R[0].solution_value(), R[1].solution_value()]

if __name__ == '__main__':
  print(XNetwork())
