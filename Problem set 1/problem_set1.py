import numpy as np
import matplotlib.pyplot as plt

# Note: all functions must return NUMPY vectors or matrices
def question1(A, B):
    '''
    Use a foor loop to multiply these matrices
    Arguments:
        A: a matrix with shape (n,m)
        B: a matrix with shape (m,l)
    Returns:
        C: a matrix with shape (n,l)

    '''
    # your code here
    
    C = [[0 for row in range(len(A))] for col in range(len(B[0]))]
    for i in range(len(C)):
        for j in range(len(C[0])):
            for k in range(len(B)):
                C[i][j] += A[i][k]*B[k][j]
    return C
    pass

def question2(A, B):
    '''
    Use a foor loop to transpose and add both matrices (transpose both matrices first and then add them)
    Arguments:
        A: a matrix with shape (n,n)
        B: a matrix with shape (n,n)
    Returns:
        C: a matrix with shape (n,n)

    '''
    # your code here
    pass

def question3(A, b):
    '''
    Solve the equation Ax = b
    Arguments:
        A: a matrix with shape (n,n)
        b: a vector with shape (n,1)
    Returns:
        x: a vector with shape (n,1) if the equation has one solution
        0: if the equation has no solution or has infinite solutions

    '''
    # your code here
    pass

def question4(A):
    '''
    Compute the eigenvalues of A
    Arguments:
        A: a matrix with shape (n,n)
    Returns:
        C: the inverse of A if the real part of all eigenvalues of A are negative. Shape (n,n)
        0: otherwise

    '''
    # your code here
    pass

def question5(N = 10, deltaT = 0.01):
    '''
    Integrate the following function from 0 to N seconds using Euler integration with x(0) = 1
    and time step of deltaT seconds
    x' = -2x^3 + sin(0.5t)x
    Arguments:
        N: Number of seconds to integrate
        deltaT: time step for integration
        
    Returns:
        x: a vector with shape (N/deltaT,1)

    '''
    # your code here
    pass

if __name__ == '__main__':
    # you can use the main function to test your functions. Not graded.
 A= [[1,2],[1,2]]
B=[[3 ,4],[3,4]]

result=question1(A,B)
print(result)
pass
