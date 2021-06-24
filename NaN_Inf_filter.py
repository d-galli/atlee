import numpy as np

A = np.array([[1, np.Inf, 3, 0, 3, np.NaN ]])

print("Input: \n")
print(A)

where_are_NaNs = np.isnan(A)

#A[where_are_NaNs] = 0
#A[where_are_Infs] = 0
#print("Output 1: \n")
#print(A)

not_NaN = ~ where_are_NaNs
A = A[not_NaN]

where_are_Infs = np.isinf(A)
not_Inf = ~ where_are_Infs
A = A[not_Inf]
print("Output 2: \n")
print(A)