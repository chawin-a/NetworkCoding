from __future__ import print_function
n = int(input())
f = open('input/Line'+str(n-1)+'.in', 'w')
print(n, file=f) # Nodes
print(n-1, file=f) # Edges
for i in range(n-1):
    print('%d %d 0.1 0.05' % (i, i+1), file=f)
print(1, file=f)
print(0, file=f)
print(1, file=f)
print(n-1, file=f)
print(1, file=f)
print('%d %d' % (0, n-1), file=f)
print(0, file=f)
print(0, file=f)
f.close()
f = open('input/Line'+str(n-1)+'.pos', 'w')
for i in range(n):
    print('%d %d' % (i, 0), file=f)
f.close()