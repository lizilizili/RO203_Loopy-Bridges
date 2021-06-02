# This file contains methods to solve an instance (heuristically or with CPLEX)
using CPLEX

include("io.jl")

function cplexSolve(t::Array{Int64, 2})

    # Create the model
    r = direct_model(CPLEX.Optimizer())

    # Allow for some impreciseness in the solution
    TOL = 0.00001

    n=size(t,1)
    m=size(t,2)

    n_node=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                n_node=n_node+1
            end
        end
    end

    list_node=zeros(Int64, n_node, 2)

    f=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                f=f+1
                list_node[f,1]=i
                list_node[f,2]=j
            end
        end
    end



    #y[i,j,k,l]=0 si i et j sont connectés, et 1 sinon
    @variable(r, y[1:n_node, 1:n_node], Bin)

    #x[i,j,k,l] le nombre de ponts entre i et j
    @variable(r, 0<=x[1:n_node, 1:n_node]<=2, Int)


    #contrainte 1: A ET B ADJACENTS SONT CONNECTES: si oui y[i,j,k,l]=1 sinon 0 , A de coord(i,j) et B de coord (k,l)
    for i in 1:n_node
        for j in 1:n_node
                    if adjacent(list_node,i,j,t)==0
                        @constraint(r, y[i,j]==0)
                    end
        end
    end

    #contrainte de symétrie
    for i in 1:n_node
        for j in 1:n_node
                    @constraint(r, y[i,j]==y[j,i])
        end
    end
    for i in 1:n_node
        @constraint(r, y[i,i]==0)
    end
    for i in 1:n_node
        @constraint(r, x[i,i]==0)
    end
    for i in 1:n_node
        for j in 1:n_node
                    @constraint(r, x[i,j]==x[j,i])
        end
    end


    #contrainte 2: LE NOMBRE DE PONTS PARTANT DU NOEUD
    for i in 1:n_node
            @constraint(r, sum(x[i,j] for j in 1:n_node)==t[list_node[i,1],list_node[i,2]])
    end

    #contrainte
    for i in 1:n_node
        for j in 1:n_node
                @constraint(r,y[i,j]<=x[i,j])
                @constraint(r,x[i,j]<=2*y[i,j])
        end
    end

    @constraint(r, sum(y[i,j] for i in 1:n_node for j in 1:n_node)>=2*(n_node-1))


    # Minimizer le nombre de ponts total
    @objective(r, Min, sum(y[i,j] for i in 1:n_node for j in 1:n_node))

    function my_callback_function(cb)
        y_val = callback_value.(Ref(cb), y)
        println("In callback function, y=$y_val")

        # Allow for some impreciseness in the solution
        TOL = 0.00001

        bool, ens1,ens2=subBridge(list_node,y_val)
        e1=size(ens1,1)
        e2=size(ens2,1)
        if bool==true
            println("Solution has ens1$ens1,ens2$ens2, cut it off")
            con = @build_constraint(sum(y[ens1[i,1],ens2[j,1]] for i in 1:e1 for j in 1:e2)>=1)
            MOI.submit(r, MOI.LazyConstraint(cb), con)
        end
    end
    MOI.set(r, MOI.LazyConstraintCallback(), my_callback_function)

    # Start a chronometer
    start = time()

    # Solve the model
    optimize!(r)

    # Return:
    # 1 - true if an optimum is found
    # 2 - the resolution time
    return JuMP.primal_status(r) == JuMP.MathOptInterface.FEASIBLE_POINT, x, y,time() - start

end





function subBridge(list_node::Array{Int64,2} , y::Array{Float64, 2})

    n_node=size(list_node,1)
    ens1=Array{Int64}(undef, 0, 1)
    mark=Array{Int64}(undef, 1, n_node)

    total=0

    for i in 1:n_node
        if list_node[i]!=0
            ens1=vcat(ens1,[i])
            mark[i]=1
        end
    end

    flag=true
    while flag
        flag=false
        for i in 1:n_node
            for j in 1:n_node
                if mark[i]==1 && mark[j]!=1 && y[i,j]==1
                    ens1=vcat(en1,[j])
                    mark[j]=1
                    flag=true
                end
            end
        end
    end

    #le nombre de noeuds marqué
    for a in 1:n_node
        total=mark[a]+total
    end

    if total==n_node
        return false, [], []
    else
        ens2=Array{Int64}(undef, 0, 1)
        for i in 1:n_node
                if (mark[i]!=1)
                    ens2=vcat(ens1,[i])
                    mark[i]=1
                end
        end

        flag=true
        while flag
            flag=false
            for i in 1:n
                for j in 1:m
                    if mark[i]==1 && y[i,j]==1
                            ens2=vcat(en1,[j])
                            mark[j]=1
                            flag=true
                    end
                end
            end
        end

        return true, ens1,en2
    end
end



#2 noeus sont voisins et peuvent être connectés (true si oui, 0 sinon)
function Noeudconnected(list_node::Array{Int64,2}, i::Int64, j::Int64)
    inter= false
    n_node=size(list_node,1)
    if adjacent(list_node, i, j)!=0
        inter=true
    end
    return inter
end

#2 ponts [i,j,k,l] et [o,p,q,r] qui peuvent s'intersecter (true si oui, 0 sinon)
function BridgeInter(list_node::Array{Int64,2}, i::Int64, j::Int64, k::Int64, l::Int64 )
    n_node=size(list_node,1)
    h= false

    #les noeuds sont dans le tableau
    while (1<=i<=n_node) && (1<=j<=n_node) && (1<=k<=n_node) && (1<=l<=n_node)
        #les 4 noeuds forment 2 ponts [A,B] [C,D] avec A(i,j), B(k,l), C(o,p), D(q,r)
        if (Noeudconnected(t,i,j)==true) && (Noeudconnected(t,k,l)==true)
            #si [i,j] est vertical et [k,l] est horizontal
            if list_node[i,2]==list_node[j,2] && list_node[k,1]==list_node[l,1]
                #if ((r>j) && (p<j)) || ((p<j) && (r<j))
                if (list_node[l,2]>list_node[i,2] && list_node[k,2]<list_node[i,2])
                    h=true
                elseif list_node[k,2]<list_node[i,2] && list_node[l,2]<list_node[i,2]
                    h=true
                end
            #si [i,j] est horizontal et [o,p] est vertical
            elseif (list_node[i,1]==list_node[j,1]) && (list_node[k,2]==list_node[l,2])
                #if ((l>p) && (j<p)) || ((j>p) && (l<p))
                if ((list_node[j,2]>list_node[k,2]) && (list_node[i,2]<list_node[k,2])) || ((list_node[i,2]>list_node[k,2]) && (list_node[j,2]<list_node[k,2]))
                    h=true
                end
            #si les 2 ponts sont horizontaux
            #elseif (i==k) && (i==o) && (o==q)
            elseif (list_node[i,1]==list_node[j,1]) && (list_node[i,1]==list_node[k,1]) && (list_node[k,1]==list_node[l,1])
                #if (l<p && l<r && j<p && j<r) || (r<j && r<l && p<j && p<l)
                if ((list_node[j,2]<list_node[k,2]) && (list_node[j,2]<list_node[l,2]) && (list_node[i,2]<list_node[k,2]) && (list_node[i,2]<list_node[l,2]))
                    h=false
                elseif ((list_node[l,2]<list_node[i,2]) && (list_node[l,2]<list_node[j,2]) && (list_node[k,2]<list_node[i,2]) && (list_node[k,2]<list_node[j,2]))
                    h=false
                end
            #si les 2 ponts sont verticaux
            #elseif (p==r) && (j==l) &&(j==p)
            elseif (list_node[k,2]==list_node[l,2]) && (list_node[i,2]==list_node[j,2]) &&(list_node[i,2]==list_node[k,2])
                if (list_node[i,1]<list_node[k,1]<list_node[j,1])
                    h=true
                elseif (list_node[j,1]<list_node[k,1]<list_node[i,1])
                    h=true
                elseif (list_node[i,1]<list_node[l,1]<list_node[j,1])
                    h=true
                elseif (list_node[j,1]<list_node[l,1]<list_node[i,1])
                    h=true
                end
            end
        end
    end
    return h
end


function adjacent(list_node::Array{Int64,2}, i::Int64, j::Int64,t::Array{Int64, 2})
    h=0
    n_node=size(list_node,1)
    s=0

        #t[k,l] voisin à droite, h=1
        if (list_node[j,1]==list_node[i,1]) && (list_node[j,2]>list_node[i,2])
            s=list_node[i,2]+1
            while s<=list_node[j,2]-1 && t[list_node[i,1],s]==0
                h=h+1
                s=s+1
            end
            if h<(list_node[j,2]-1)-(list_node[i,2]+1)+1
                h=0
            else
                h=1
            end

        #j voisin à gauche de i, h=2
        elseif (list_node[j,1]==list_node[i,1]) && (list_node[j,2]<list_node[i,2])
            s=list_node[j,2]+1
            while s<=list_node[i,2]-1 && t[list_node[i,1],s]==0
                h=h+1
                s=s+1
            end
            if h<(list_node[i,2]-1)-(list_node[j,2]+1)+1
                h=0
            else
                h=2
            end

            #j voisin en haut de i, h=3
        elseif (list_node[i,2]==list_node[j,2]) && (list_node[i,1]>list_node[j,1])
            s=list_node[j,1]+1
            while s<=list_node[i,1]-1 && t[s,list_node[i,2]]==0
                h=h+1
                s=s+1
            end
            if h<(list_node[i,1]-1)-(list_node[j,1]+1)+1
                h=0
            else
                h=3
            end
            #t[k,l] voisin en bas, h=4
        elseif (list_node[i,2]==list_node[j,2]) && (list_node[j,1]>list_node[i,1])
            s=list_node[i,1]+1
            while s<=list_node[j,1]-1 && t[s,list_node[i,2]]==0
                h=h+1
                s=s+1
            end
            if h<(list_node[j,1]-1)-(list_node[i,1]+1)+1
                h=0
            else
                h=4
            end
        end
        return h

end



function solveDataSet()

    dataFolder = "../data/"
    resFolder = "../res/"

    resolutionMethod = ["cplex"]
    resolutionFolder = resFolder .* resolutionMethod

    for folder in resolutionFolder
        if !isdir(folder)
            mkdir(folder)
        end
    end

    global isOptimal = false
    global solveTime = -1

    # For each input file
    # (for each file in folder dataFolder which ends by ".txt")
    for file in filter(x->occursin(".txt", x), readdir(dataFolder))

        println("-- Resolution of ", file)
        t = readInputFile(dataFolder * file)

        # For each resolution method
        for methodId in 1:size(resolutionMethod, 1)

            outputFile = resolutionFolder[methodId] * "/" * file

            # If the input file has not already been solved by this method
            if !isfile(outputFile)

                fout = open(outputFile, "w")

                resolutionTime = -1
                isOptimal = false

                # If the method is cplex
                if resolutionMethod[methodId] == "cplex"

                    # Solve it and get the results
                    isOptimal, x, y, resolutionTime = cplexSolve(t)

                    # Also write the solution (if any)
                    if isOptimal
                        writeSolution(fout, JuMP.value.(x),t)
                    end

                # If the method is one of the heuristics
                else

                    isSolved = false
                    solution = []

                    # Start a chronometer
                    startingTime = time()

                    # While the grid is not solved and less than 100 seconds are elapsed
                    while !isOptimal && resolutionTime < 100
                        print(".")

                        isOptimal, solution = heuristicSolve(t, resolutionMethod[methodId] == "heuristique2")

                        # Stop the chronometer
                        resolutionTime = time() - startingTime
                    end

                    println("")

                    # Write the solution (if any)
                    if isOptimal
                        writeSolution(fout, solution)
                    end
                end

                println(fout, "solveTime = ", resolutionTime)
                println(fout, "isOptimal = ", isOptimal)
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
Heuristically solve an instance
"""
function heuristicSolve(t::Array{Int64, 2})

	tCopy = deepcopy(t)
	n=size(t,1)
    m=size(t,2)

    n_node=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                n_node=n_node+1
            end
        end
    end

    list_node=zeros(Int64, n_node, 2)

    f=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                f=f+1
                list_node[f,1]=i
                list_node[f,2]=j
            end
        end
    end

	#while (Solved(t,tCopy)==false)
end
