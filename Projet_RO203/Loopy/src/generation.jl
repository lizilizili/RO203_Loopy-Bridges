# This file contains methods to generate a data set of instances (i.e., sudoku grids)
include("io.jl")

"""
Generate an n*n grid with a given density

Argument
- n: size of the grid
- density: percentage in [0, 1] of initial values in the grid
"""
function generateInstance(n::Int64, density::Float64)

    t_complete = zeros(Int64, n, n)
    t = -1*ones(Int64, n, n)
    i = 1
    ## generate a graphe
    genrateByColoration(t_complete)
        # While the grid is valid and the required number of cells is not filled
    while findmax(t)[1]<1
        t = -1*ones(Int64, n, n)
        while  i < (n*n*density)

            # Randomly select a cell and a value
            l = ceil.(Int, n * rand())
            c = ceil.(Int, n * rand())

            if t[l,c]==-1
                i+=1
                t[l,c]=t_complete[l,c]
            end
        end
    end
    return t

end

"""
Generate an n*n grid by coloring cell adjacent

Argument
- t_complete : the complet table with t_complete[i,j] the number of edges around
"""
function genrateByColoration(t_complete::Array{Int64, 2})
    n=size(t_complete,1)
    g = zeros(Int64, n, n)
    l = ceil.(Int, n * rand())
    c = ceil.(Int, n * rand())
    g[l,c]=1
    cells = ceil.(Int, (n*n/4) *rand()+n*n/2)
    i=1
    while i<cells
        l = ceil.(Int, n * rand())
        c = ceil.(Int, n * rand())
        if isGridValid(g,l,c)
            i+=1
            g[l,c]=1
        end
    end
    for l in 1:n
        for r in 1:n
            if l>1
                if g[l,r]!=g[l-1,r] t_complete[l,r]+=1 end
            else
                if g[l,r]!=0 t_complete[l,r]+=1 end
            end
            if l<n
                if g[l,r]!=g[l+1,r] t_complete[l,r]+=1 end
            else
                if g[l,r]!=0 t_complete[l,r]+=1 end
            end
            if r>1
                if g[l,r]!=g[l,r-1] t_complete[l,r]+=1 end
            else
                if g[l,r]!=0 t_complete[l,r]+=1 end
            end
            if r<n
                if g[l,r]!=g[l,r+1] t_complete[l,r]+=1 end
            else
                if g[l,r]!=0 t_complete[l,r]+=1 end
            end
        end
    end
end

"""
Test if the cell [l,r] can be colored

Argument
- g : n+1*n+1 g[i,j]==1 if colored
- l,r : position of test
"""

function isGridValid(g::Array{Int64, 2},l::Int64,r::Int64)
    n=size(g,1)
    flag = false
    if l>1 && g[l-1,r]==1 flag=true end
    if l<n && g[l+1,r]==1 flag=true end
    if r>1 && g[l,r-1]==1 flag=true end
    if r<n && g[l,r+1]==1 flag=true end
    if !flag return false end

    if l>1 && g[l-1,r]==0
        i=l-1
        j=r
        sum=0
        if i>1 && g[i-1,j]==1 sum+=1 end
        if i<n && g[i+1,j]==1 sum+=1 end
        if j>1 && g[i,j-1]==1 sum+=1 end
        if j<n && g[i,j+1]==1 sum+=1 end
        if sum==4 return false end
    end
    if l<n && g[l+1,r]==0
        i=l+1
        j=r
        sum=0
        if i>1 && g[i-1,j]==1 sum+=1 end
        if i<n && g[i+1,j]==1 sum+=1 end
        if j>1 && g[i,j-1]==1 sum+=1 end
        if j<n && g[i,j+1]==1 sum+=1 end
        if sum==4 return false end
    end
    if r>1 && g[l,r-1]==0
        i=l
        j=r-1
        sum=0
        if i>1 && g[i-1,j]==1 sum+=1 end
        if i<n && g[i+1,j]==1 sum+=1 end
        if j>1 && g[i,j-1]==1 sum+=1 end
        if j<n && g[i,j+1]==1 sum+=1 end
        if sum==4 return false end
    end
    if r<n && g[l,r+1]==0
        i=l
        j=r+1
        sum=0
        if i>1 && g[i-1,j]==1 sum+=1 end
        if i<n && g[i+1,j]==1 sum+=1 end
        if j>1 && g[i,j-1]==1 sum+=1 end
        if j<n && g[i,j+1]==1 sum+=1 end
        if sum==4 return false end
    end
    return true
end


"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    # For each grid size considered
    for size in [4,6,9]

        # For each grid density considered
        for density in [0.4,0.6,0.7]

            # Generate 10 instances
            for instance in 1:10

                fileName = "../data/instance_t" * string(size) * "_d" * string(density) * "_" * string(instance) * ".txt"

                if !isfile(fileName)
                    println("-- Generating file " * fileName)
                    saveInstance(generateInstance(size, density), fileName)
                end
            end
        end
    end

end
