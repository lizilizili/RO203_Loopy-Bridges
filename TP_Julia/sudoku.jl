using JuMP
using CPLEX

function sudoku(t::Array{Int, 2})

    # Size of the grid
    n = size(t, 1)

    # Create the model
    m = Model(with_optimizer(CPLEX.Optimizer))

    ### Objective
    @objective(m, Max, 1)
    
    ### Variables
    # x[i, j, k] = 1 if cell (i, j) has value k
    @variable(m, x[1:n, 1:n, 1:n], Bin)

    ### Constraints

    # Get the size of a block
    blockSize = round.(Int, sqrt(n))

    # TODO

    ### Solve the problem
    optimize!(m)

    ### Display the solution
    # TODO

end

t = [
0 7 0 2 0 3 0 1 0;
3 0 0 0 0 0 0 0 0;
0 0 0 0 0 0 2 0 0;
0 0 0 0 0 0 0 0 0;
0 0 0 0 0 0 0 0 0;
0 0 0 0 0 0 0 0 2;
2 0 0 0 0 0 0 0 0;
0 0 0 0 0 0 0 0 0;
0 0 0 0 0 0 0 0 0]

sudoku(t)

