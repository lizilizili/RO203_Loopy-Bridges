# This file contains methods to solve an instance (heuristically or with CPLEX)
using CPLEX

include("generation.jl")

TOL = 0.00001

"""
Solve an instance with CPLEX
"""
function cplexSolve(t::Array{Int, 2})

    n = size(t, 1)

    # Create the model
    m = direct_model(CPLEX.Optimizer())

    # x[i, j] = 1 if the left edge of (i,j) is chosen.
    @variable(m, x[1:n, 1:n+1], Bin)
    # y[i, j] = 1 if the up edge of (i,j) is chosen.
    @variable(m, y[1:n+1, 1:n], Bin)
    # p[i, j] = 1 if the circle traverse the up-left node of (i,j).
    @variable(m, p[1:n+1, 1:n+1], Bin)
    # Set the number of edges around each cell
    for l in 1:n
        for c in 1:n
            if t[l, c] != -1
                @constraint(m, x[l, c]+x[l, c+1]+y[l, c]+y[l+1, c] == t[l,c])
            end
            if t[l, c] == 3
                @constraint(m, [i in l:l+1, j in c:c+1],p[i,j]==1)
            end
        end
    end

    # there is less than one line pass the node(i,j)
    @constraint(m, [i in 2:n, j in 2:n], x[i, j]+y[i,j]+x[i-1, j]+y[i,j-1] == 2*p[i,j])

    @constraint(m, x[1, 1]+y[1, 1] == 2*p[1, 1])
    @constraint(m, x[1, n+1]+y[1, n] == 2*p[1, n+1])
    @constraint(m, x[n, 1]+y[n+1, 1] == 2*p[n+1, 1])
    @constraint(m, x[n, n+1]+y[n+1, n] == 2*p[n+1, n+1])

    @constraint(m, [i in 2:n], x[i, 1]+y[i,1]+x[i-1, 1] == 2* p[i,1])
    @constraint(m, [i in 2:n], x[i, n+1]+x[i-1, n+1]+y[i,n] ==  2*p[i,n+1])
    @constraint(m, [j in 2:n], x[1, j]+y[1,j]+y[1,j-1] ==  2*p[1,j])
    @constraint(m, [j in 2:n], y[n+1,j]+x[n, j]+y[n+1,j-1] ==  2*p[n+1,j])

    # Minimize the number of node (reduce the problem symmetry)
    @objective(m, Min, sum(p[i,j] for i in 1:n+1 for j in 1:n+1))

    function my_callback_function(cb)
        x_val = callback_value.(Ref(cb), x)
        y_val = callback_value.(Ref(cb), y)
        p_val = callback_value.(Ref(cb), p)
        #println("In callback function, x=$x_val, y=$y_val,p=$p_val")


        # Allow for some impreciseness in the solution
        TOL = 0.00001

        subloop,loopChainx,loopChainy = checkSubLoop(x_val,y_val,p_val)
        n=size(loopChainx)[1]+size(loopChainy)[1]

        # if there is a subloop, on cut it off
        if subloop
            #println("Solution has subloop$subloop, $loopChainx,$loopChainy, cut it off")
            con = @build_constraint(sum(x[loopChainx[i,1],loopChainx[i,2]] for i in 1:size(loopChainx)[1])+sum(y[loopChainy[i,1],loopChainy[i,2]] for i in 1:size(loopChainy)[1]) <= n  )
            MOI.submit(m, MOI.LazyConstraint(cb), con)
        end
    end

    MOI.set(m, MOI.LazyConstraintCallback(), my_callback_function)
    # Start a chronometer
    start = time()

    # Solve the model
    optimize!(m)

    # Return:
    # 1 - true if an optimum is found
    # 2 - the solution
    # 3 - the resolution time
    return JuMP.primal_status(m) == JuMP.MathOptInterface.FEASIBLE_POINT, x,y,p,time() - start

end

"""
Heuristically solve an instance
"""
function heuristicSolve(t::Array{Int, 2},start_time::Float64)
    n = size(t, 1)
    x = -1*ones(Float64,n,n+1)
    y = -1*ones(Float64,n+1,n)
    p = -1*ones(Float64,n+1,n+1)
    node = zeros(Int64,n+1,n+1)
    grid1 = zeros(Int64,n,n)
    initialisation(x,y,p,t)

    fillByNode(x,y,p,node)
    fillByGrid(x,y,p,grid1,t)
    #find not connected edge
    index = findPossibleBranch(x,y,p,node,grid1,t)
    if index[1]!=0
        isOptimal,xSol,ySol,pSol = progressLoop(x,y,p,node,grid1,t,start_time)
        return isOptimal,xSol,ySol,pSol
    #if not, find a filled grid to start
    else
        idgrid,directions=findPossibleGrid(x,y,t)
        i=idgrid[1]
        j=idgrid[2]
        for d in directions
            xCopy = deepcopy(x)
            yCopy = deepcopy(y)
            pCopy = deepcopy(p)
            nodeCopy = deepcopy(node)
            gridCopy = deepcopy(grid1)
            if d==1
                xCopy[i,j]=1
                pCopy[i,j]=1
                pCopy[i+1,j]=1
            end
            if d==2
                yCopy[i,j]=1
                pCopy[i,j]=1
                pCopy[i,j+1]=1
            end
            if d==3
                xCopy[i,j+1]=1
                pCopy[i,j+1]=1
                pCopy[i+1,j+1]=1
            end
            if d==4
                yCopy[i+1,j]=1
                pCopy[i+1,j]=1
                pCopy[i+1,j+1]=1
            end
            isOptimal,xSol,ySol,pSol = progressLoop(xCopy,yCopy,pCopy,nodeCopy,gridCopy,t,start_time)
            if isOptimal
                return isOptimal,xSol,ySol,pSol
            end
        end
    end
    return isOptimal,xSol,ySol,pSol
end

"""
Solve all the instances contained in "../data" through CPLEX and heuristics

The results are written in "../res/cplex" and "../res/heuristic"

Remark: If an instance has previously been solved (either by cplex or the heuristic) it will not be solved again
"""
function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"

    # Array which contains the name of the resolution methods
    #resolutionMethod = ["cplex"]
    resolutionMethod = ["cplex", "heuristique"]

    # Array which contains the result folder of each resolution method
    resolutionFolder = resFolder .* resolutionMethod

    # Create each result folder if it does not exist
    for folder in resolutionFolder
        if !isdir(folder)
            mkdir(folder)
        end
    end

    global isOptimal = false
    global solveTime = -1

    # For each instance
    # (for each file in folder dataFolder which ends by ".txt")
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))

        println("-- Resolution of ", file)
        t = readInputFile(dataFolder * file)

        # For each resolution method
        for methodId in 1:size(resolutionMethod, 1)

            outputFile = resolutionFolder[methodId] * "/" * file

            # If the instance has not already been solved by this method
            if !isfile(outputFile)

                fout = open(outputFile, "w")

                resolutionTime = -1
                isOptimal = false

                # If the method is cplex
                if resolutionMethod[methodId] == "cplex"


                    # Solve it and get the results
                    isOptimal,x,y,p, resolutionTime = cplexSolve(t)

                    # If a solution is found, write it
                    if isOptimal

                        writeSolution(fout, JuMP.value.(x),JuMP.value.(y),t)

                    end

                # If the method is one of the heuristics
                else

                    isSolved = false

                    # Start a chronometer
                    startingTime = time()

                    # While the grid is not solved and less than 100 seconds are elapsed
                    while !isOptimal && resolutionTime < 100
                        print(".")

                        # Solve it and get the results
                        isOptimal, x,y,p = heuristicSolve(t,startingTime)

                        # Stop the chronometer
                        resolutionTime = time() - startingTime

                    end
                    println("")
                    # Write the solution (if any)
                    if isOptimal
                        writeSolution(fout, x,y,t)
                    end
                end

                println(fout, "solveTime = ", resolutionTime)
                println(fout, "isOptimal = ", isOptimal)

                # TODO

                #println("In file resolution.jl, in method solveDataSet(), TODO: write the solution in fout")
                close(fout)
            end


            # Display the results obtained with the method on the current instance
            include(outputFile)
            println(resolutionMethod[methodId], " optimal: ", isOptimal)
            println(resolutionMethod[methodId], " time: " * string(round(solveTime, sigdigits=2)) * "s\n")
        end
    end
end
"""
true if there is a subloop
"""
function checkSubLoop(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2})
    s = size(p)[1]
    n=0
    # g:[n+1*n+1] statu of node
    g=zeros(Int64,s,s)

    # n: number of used nodes
    for i in 1:s
        for j in 1:s
            if p[i,j]==1 n+=1 end
        end
    end

    #n_loop: we used n_loop to identify defferent loops
    n_loop=0
    while true
        n_loop+=1
        subLoopx = Array{Int64}(undef, 0 , 2)
        subLoopy = Array{Int64}(undef, 0 , 2)

        #find a node to start
        p_start=[0 0]
        flag = true
        i=1
        j=1
        while p_start==[0 0] && i<=s &&j<=s
            if p[i,j]==1 && g[i,j]==0
                p_start =[i j]
                g[i,j]=n_loop
            end
            if i<s
                i+=1
            else
                i=1
                j+=1
            end
        end
        #println(p_start)
        #println(n_loop)

        #return if all node noted
        if p_start==[0 0]
            return false,[],[]
        end

        p_present =  p_start
        last_direction=0

        #put adjacent node in subloop
        while flag
            i=p_present[1]
            j=p_present[2]
            flag = false
            if (i>1 && p[i-1,j] == 1 && x[i-1,j] == 1)
                if g[i-1,j] == 0
                    flag= true
                    g[i-1,j] = n_loop
                    p_present= [i-1 j]
                    subLoopx=vcat(subLoopx,[i-1 j])
                    i=i-1
                    last_direction=1
                else
                    #return the subloop if it exist
                    if size(subLoopx)[1]+size(subLoopy)[1]!=n-1 && last_direction!=2 && g[i-1,j]== n_loop
                        return true,subLoopx,subLoopy
                    end
                end
            end
            #println(last_direction)
            if (i<s && p[i+1,j] == 1 && x[i,j] == 1)
                if g[i+1,j] == 0
                    flag= true
                    g[i+1,j] = n_loop
                    p_present= [i+1 j]
                    subLoopx=vcat(subLoopx,[i j])
                    i=i+1
                    last_direction=2
                else
                    if size(subLoopx)[1]+size(subLoopy)[1]!=n-1 && last_direction!=1 && g[i+1,j]== n_loop
                        return true,subLoopx,subLoopy
                    end
                end
            end
            #println(last_direction)
            if (j>1 && p[i,j-1] == 1 && y[i,j-1] == 1)
                if  g[i,j-1] == 0
                    flag= true
                    g[i,j-1] = n_loop
                    p_present= [i j-1]
                    subLoopy=vcat(subLoopy,[i j-1])
                    j-=1
                    last_direction=3
                else
                    if size(subLoopx)[1]+size(subLoopy)[1]!=n-1 && last_direction!=4 && g[i,j-1]== n_loop
                        return true,subLoopx,subLoopy
                    end
                end
            end
            #println(last_direction)
            if (j<s && p[i,j+1] == 1 && y[i,j] == 1)
                if g[i,j+1] == 0
                    flag= true
                    g[i,j+1] =  n_loop
                    p_present= [i j+1]
                    subLoopy=vcat(subLoopy,[i j])
                    j+=1
                    last_direction=4
                else
                    if size(subLoopx)[1]+size(subLoopy)[1]!=n-1 && last_direction!=3 && g[i,j+1]== n_loop
                        return true,subLoopx,subLoopy
                    end
                end
                #println(last_direction)
            end
        end
        if size(subLoopx)[1]+size(subLoopy)[1]==n-1
            return false,[],[]
        end
    end

    return false,[],[]

end
"""
true if there is a dead end
"""
function checkDeadEnd(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2})
    s = size(p)[1]
    g = zeros(Int64, s, s)
    for i in 1:s
        for j in 1:s
            if  i<s && x[i,j]!=0
                g[i,j]+=1
                g[i+1,j]+=1
            end

            if  j<s && y[i,j]!=0
                g[i,j]+=1
                g[i,j+1]+=1
            end
        end
    end
    ## Check dead Point
    for i in 1:s
        for j in 1:s
            if  p[i,j]==1 && g[i,j]<2
                return true
            end
        end
    end

    return false
end

"""
true if there is a wrong grid
"""

function checkWrongGrid(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},t::Array{Int64, 2})
    s = size(t)[1]
    g = zeros(Int64, s, s)
    for i in 1:s
        for j in 1:s
            if t[i,j]!=-1
                sum=0
                sum_0=0
                if x[i,j]==1 sum+=1 end
                if x[i,j+1]==1 sum+=1 end
                if y[i,j]==1 sum+=1 end
                if y[i+1,j]==1 sum+=1 end
                if x[i,j]==0 sum_0+=1 end
                if x[i,j+1]==0 sum_0+=1 end
                if y[i,j]==0 sum_0+=1 end
                if y[i+1,j]==0 sum_0+=1 end
                if sum>t[i,j] return true end
                if sum_0>4-t[i,j]  return true end
            end
        end
    end
    return false
end

"""
Initialize for case : 1/3 in corner
"""
function initialisation(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},t::Array{Int64, 2})
    s = size(t)[1]
    newArrayFilled=false

    if t[1,1]==1 p[1,1]=0 end
    if t[1,s]==1 p[1,s+1]=0 end
    if t[s,1]==1 p[s+1,1]=0 end
    if t[s,s]==1 p[s+1,s+1]=0 end
    if t[1,1]==3 p[1,1]=1 end
    if t[1,s]==3 p[1,s+1]=1 end
    if t[s,1]==3 p[s+1,1]=1 end
    if t[s,s]==3 p[s+1,s+1]=1 end
end

"""
Go forward by checking constrains of node
"""
function fillByNode(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},node::Array{Int64, 2})
    s = size(p)[1]
    newArrayFilled=false
    for i in 1:s
        for j in 1:s
            if  node[i,j]==0
                sum_1=0
                sum_n=0

                if i<s && x[i,j]==1 sum_1+=1 end
                if i<s && x[i,j]==-1 sum_n+=1 end
                if i>1 && x[i-1,j]==-1 sum_n+=1 end
                if i>1 && x[i-1,j]==1 sum_1+=1 end
                if j<s && y[i,j]==-1 sum_n+=1 end
                if j<s && y[i,j]==1 sum_1+=1 end
                if j>1 && y[i,j-1]==-1 sum_n+=1 end
                if j>1 && y[i,j-1]==1 sum_1+=1 end
                #println("sum_n in $i,$j is $sum_n")
                #println("sum_1 in $i,$j is $sum_1")
                if sum_n+sum_1==0 node[i,j]==1 end
                if sum_n==1 && sum_1==0
                    if i<s  x[i,j]=0 end
                    if i>1  x[i-1,j]=0 end
                    if j<s  y[i,j]=0 end
                    if j>1  y[i,j-1]=0 end
                    p[i,j]=0
                    node[i,j]==1
                end
                if sum_1==2
                    if i<s &&x[i,j]!=1  x[i,j]=0 end
                    if i>1 && x[i-1,j]!=1  x[i-1,j]=0 end
                    if j<s &&y[i,j]!=1  y[i,j]=0 end
                    if j>1 && y[i,j-1]!=1 y[i,j-1]=0 end
                    node[i,j]=1
                elseif  sum_n+sum_1==2 && p[i,j]==1
                    if i<s && x[i,j]==-1
                        x[i,j]=1
                        p[i,j]=1
                        p[i+1,j]=1
                    end
                    if i>1 && x[i-1,j]==-1
                        x[i-1,j]=1
                        p[i-1,j]=1
                        p[i,j]=1
                    end
                    if j<s &&y[i,j]==-1
                        y[i,j]=1
                        p[i,j]=1
                        p[i,j+1]=1

                    end
                    if j>1 && y[i,j-1]==-1
                        y[i,j-1]=1
                        p[i,j]=1
                        p[i,j-1]=1
                    end
                    node[i,j]=1

                end
                if node[i,j]==1 newArrayFilled=true end
            end
            if p[i,j]==0 && node[i,j]==0
                if i<s && x[i,j]==-1  x[i,j]=0 end
                if i>1 && x[i-1,j]==-1  x[i-1,j]=0 end
                if j<s &&y[i,j]==-1  y[i,j]=0 end
                if j>1 && y[i,j-1]==-1 y[i,j-1]=0 end
                node[i,j]=1
                newArrayFilled=true
            end
        end
    end
    return newArrayFilled
end

"""
Go forward by checking constrains of cell
"""
function fillByGrid(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},grid::Array{Int64, 2},t::Array{Int64, 2})
    s = size(t)[1]
    newArrayFilled=false
    for i in 1:s
        for j in 1:s
            if t[i,j]!=-1 && grid[i,j]==0
                sum_1=0
                sum_0=0
                if x[i,j]==0 sum_0+=1 end
                if x[i,j]==1 sum_1+=1 end
                if x[i,j+1]==0 sum_0+=1 end
                if x[i,j+1]==1 sum_1+=1 end
                if y[i,j]==0 sum_0+=1 end
                if y[i,j]==1 sum_1+=1 end
                if y[i+1,j]==0 sum_0+=1 end
                if y[i+1,j]==1 sum_1+=1 end
                #println("sum_0 in $i,$j is $sum_0")
                #println("sum_1 in $i,$j is $sum_1")
                if sum_0==4-t[i,j]
                    if x[i,j]==-1
                        x[i,j]=1
                        p[i,j]=1
                        p[i+1,j]=1
                    end
                    if x[i,j+1]==-1
                        x[i,j+1]=1
                        p[i,j+1]=1
                        p[i+1,j+1]=1
                    end
                    if y[i,j]==-1
                        y[i,j]=1
                        p[i,j]=1
                        p[i,j+1]=1
                    end
                    if y[i+1,j]==-1
                        y[i+1,j]=1
                        p[i+1,j]=1
                        p[i+1,j+1]=1
                    end
                    grid[i,j]=1
                elseif sum_1==t[i,j]
                    if x[i,j]!=1  x[i,j]=0 end
                    if x[i,j+1]!=1  x[i,j+1]=0 end
                    if y[i,j]!=1  y[i,j]=0 end
                    if y[i+1,j]!=1 y[i+1,j]=0 end
                    grid[i,j]=1
                end
                if grid[i,j]==1 newArrayFilled=true end
            end
        end
    end
    return newArrayFilled
end

"""
Deep-search
"""
function progressLoop(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},node::Array{Int64, 2},grid::Array{Int64, 2},t::Array{Int64, 2},start_time::Float64)
    #println(x,y,p)
    #solutionGraph(x,y,t)
    if time()-start_time>100 return false,x,y,p end
    flag=true
    while flag
        flag = false
        if fillByNode(x,y,p,node)  flag=true end
        #println(x,y,p)
        #solutionGraph(x,y,t)
        if fillByGrid(x,y,p,grid,t) flag=true end
        #println(x,y,p)
        #solutionGraph(x,y,t)
    end
    #println(x,y,p)
    existSub,loop=checkSubLoop(x,y,p)
    if checkDeadEnd(x,y,p) || existSub || checkWrongGrid(x,y,p,t) return false,x,y,p end
    if endOfGame(x,y,p,t) return true,x,y,p end
    s = size(p)[1]

    index = findPossibleBranch(x,y,p,node,grid,t)
    #println(index)
    if index[1]==0 return false,x,y,p end
    i=index[1]
    j=index[2]
    xCopy = deepcopy(x)
    yCopy = deepcopy(y)
    pCopy = deepcopy(p)
    nodeCopy = deepcopy(node)
    gridCopy = deepcopy(grid)

    # try four directions
    if i<s && xCopy[i,j]==-1
         xCopy[i,j]=1
         pCopy[i,j]=1
         pCopy[i+1,j]=1
         isOptimal,xSol,ySol,pSol=progressLoop(xCopy,yCopy, pCopy,nodeCopy, gridCopy,t,start_time)
         if isOptimal
             return true ,xSol,ySol,pSol
         else
             #println("Back from x[$i,$j]")
             xCopy = deepcopy(x)
             yCopy = deepcopy(y)
             pCopy = deepcopy(p)
             nodeCopy = deepcopy(node)
             gridCopy = deepcopy(grid)
             xCopy[i,j]=0

        end
     end
    if i>1 && xCopy[i-1,j]==-1
        xCopy[i-1,j]=1
        pCopy[i-1,j]=1
        pCopy[i,j]=1
        isOptimal,xSol,ySol,pSol=progressLoop(xCopy,yCopy, pCopy,nodeCopy, gridCopy,t,start_time)
        if isOptimal
            return true ,xSol,ySol,pSol
        else
            #println("Back from x[$i-1,$j]")
            xCopy = deepcopy(x)
            yCopy = deepcopy(y)
            pCopy = deepcopy(p)
            nodeCopy = deepcopy(node)
            gridCopy = deepcopy(grid)
            xCopy[i-1,j]=0
       end
    end
    if j<s && yCopy[i,j]==-1
        yCopy[i,j]=1
        pCopy[i,j]=1
        pCopy[i,j+1]=1
        isOptimal,xSol,ySol,pSol=progressLoop(xCopy,yCopy, pCopy,nodeCopy, gridCopy,t,start_time)
        if isOptimal
            return true ,xSol,ySol,pSol
        else
            #println("from y[$i,$j]")
            xCopy = deepcopy(x)
            yCopy = deepcopy(y)
            pCopy = deepcopy(p)
            nodeCopy = deepcopy(node)
            gridCopy = deepcopy(grid)
            yCopy[i,j]=0
       end
    end
    if j>1 && yCopy[i,j-1]==-1
        yCopy[i,j-1]=1
        pCopy[i,j-1]=1
        pCopy[i,j]=1
        isOptimal,xSol,ySol,pSol=progressLoop(xCopy,yCopy, pCopy,nodeCopy, gridCopy,t,start_time)
        if isOptimal
            return true ,xSol,ySol,pSol
        else
            #println("from y[$i,$j-1]")
            xCopy = deepcopy(x)
            yCopy = deepcopy(y)
            pCopy = deepcopy(p)
            nodeCopy = deepcopy(node)
            gridCopy = deepcopy(grid)
            yCopy[i,j-1]=0
       end
    end
    return false,xCopy,yCopy,pCopy

end

"""
Find an edge which is not connected and which has the least choices
"""
function findPossibleBranch(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},node::Array{Int64, 2},grid::Array{Int64, 2},t::Array{Int64, 2})
    s = size(p)[1]
    g = zeros(Int64, s, s)
    list = Array{Int64}(undef, 0 , 2)
    for i in 1:s
        for j in 1:s
            if  i<s && x[i,j] == -1
                g[i,j]+=1
                g[i+1,j]+=1
            end

            if  j<s && y[i,j]==-1
                g[i,j]+=1
                g[i,j+1]+=1
            end
        end
    end
    min=0
    imin=1
    jmin=1
    for i in 1:s
        for j in 1:s
            if (min==0 || g[i,j]<g[imin,jmin] && g[i,j]>0 ) && p[i,j]==1
                min=g[i,j]
                imin=i
                jmin=j
            end
            if (min==0 || g[i,j] == g[imin,jmin] && g[i,j]>0 ) && p[i,j]==1
                if rand()*2>1
                    min=g[i,j]
                    imin=i
                    jmin=j
                end
            end
        end
    end
    if min==0 return [0,0] end
    #for i in 1:s
    #    for j in 1:s
    #        if min ==
    #    end
    #end
    return [imin,jmin]
end

"""
Find a cell which has the least available edges
"""
function findPossibleGrid(x::Array{Float64, 2}, y::Array{Float64, 2},t::Array{Int64, 2})
    s = size(t)[1]
    g = zeros(Int64, s, s)
    directions=[]
    for i in 1:s
        for j in 1:s
            if t[i,j]!=-1
                if x[i,j]==-1 g[i,j]+=1 end
                if x[i,j+1]==-1 g[i,j]+=1 end
                if y[i,j]==-1 g[i,j]+=1 end
                if y[i+1,j]==-1 g[i,j]+=1 end
            end
        end
    end
    min=g[1,1]
    imin=1
    jmin=1
    for i in 1:s
        for j in 1:s
            if (min==0 || g[i,j]<g[imin,jmin] && g[i,j]>0 )
                min=g[i,j]
                imin=i
                jmin=j
            end
        end
    end
    if min==0 return [0,0],[] end
    if x[imin,jmin]==-1 directions=append!(directions,1) end
    if x[imin,jmin+1]==-1 directions=append!(directions,3) end
    if y[imin,jmin]==-1 directions=append!(directions,2) end
    if y[imin+1,jmin]==-1 directions=append!(directions,4) end
    return [imin,jmin],directions
end

"""
Check if it's the end of game
"""
function endOfGame(x::Array{Float64, 2}, y::Array{Float64, 2}, p::Array{Float64, 2},t::Array{Int64, 2})
    s = size(t)[1]
    for i in 1:s
        for j in 1:s
            if t[i,j]!=-1
                sum=0
                if x[i,j]==1 sum+=1 end
                if x[i,j+1]==1 sum+=1 end
                if y[i,j]==1 sum+=1 end
                if y[i+1,j]==1 sum+=1 end
                if t[i,j]!=sum return false end
            end
        end
    end
    return true
end
