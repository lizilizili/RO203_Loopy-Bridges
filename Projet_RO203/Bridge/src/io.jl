# This file contains functions related to reading, writing and displaying a grid and experimental results

using JuMP
using Plots
import GR


function readInputFile(inputFile::String)

    # Open the input file
    datafile = open(inputFile)

    data = readlines(datafile)
    close(datafile)

    n = length(split(data[1], ","))
    m=0
    for line in data
        m=m+1
    end
    t = Array{Int64}(undef, n, m)

    lineNb = 1
    # For each line of the input file
    for line in data

        lineSplit = split(line, ",")

        if size(lineSplit, 1) == n
            for colNb in 1:m

                if lineSplit[colNb] != " "
                    t[lineNb, colNb] = parse(Int64, lineSplit[colNb])
                else
                    t[lineNb, colNb] = 0
                end
            end
        end

        lineNb += 1
    end

    return t

end


function displayGrid(t::Array{Int64, 2})

    n = size(t, 1)
    m=size(t,2)

    for i in 1:n
        for j in 1:m
            if t[i,j]!=0
                print(t[i,j],"  ")
            else
                print("   ")
            end
        end
        println()
	for j in 1:m
		print("   ")
	end
	println()
    end

end



function displaySolution( x::Array{Float64, 2},t::Array{Int64, 2},)

	n = size(t, 1)
	m = size(t, 2)
	n_bridge = size(x,1)
	n_node=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                n_node=n_node+1
            end
        end
    end
	list_node=zeros(Int64, n_node, 2)
	liasons=zeros(Int64, n,m,n,m)
	#-1 -2 si vertical -3 -4 si horizontal
	map = deepcopy(t)
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

	for i in 1:n_node-1
        for j in i+1:n_node
			if x[i,j]!=0
				o=list_node[i,1]
				p=list_node[i,2]
				k=list_node[j,1]
				l=list_node[j,2]

				if (o==k)
					if p>l
						temp=p
						p=l
						l=temp
					end
					for j1 in p:l-1
						liasons[o,j1,o,j1+1]=x[i,j]
					end
				end
				if (p==l)
					if o>k
						temp=o
						o=k
						k=temp
					end
					for i1 in o:k-1
						liasons[i1,p,i1+1,p]=x[i,j]
					end
				end
			end
		end
	end

    for l in 1:n
        s = ""
        for c in 1:m
            if t[l,c]!=0
				value=t[l,c]
                s=s*"$value"
            else
                s=s*" "
            end
			if c<m
				if liasons[l,c,l,c+1]==2
					s=s*" = "
				end
				if liasons[l,c,l,c+1]==1
					s=s*" - "
				end
				if liasons[l,c,l,c+1]==0
					s=s*"   "
				end
			end
        end
        println(s)

        s = ""
		if l<n
        	for c in 1:n
				if liasons[l,c,l+1,c]==2
					s=s*"||  "
				end
				if liasons[l,c,l+1,c]==1
					s=s*"|   "
				end
				if liasons[l,c,l+1,c]==0
					s=s*"    "
				end
        	end
			println(s)
        end

    end

end

"""
Draw a solution in a text file

"""
function writeSolution(fout::IOStream,x::Array{Float64, 2}, t::Array{Int64, 2})

    n = size(t, 1)
	m = size(t, 2)
	n_bridge = size(x,1)
	n_node=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                n_node=n_node+1
            end
        end
    end
	list_node=zeros(Int64, n_node, 2)
	liasons=zeros(Int64, n,m,n,m)
	#-1 -2 si vertical -3 -4 si horizontal
	map = deepcopy(t)
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

	for i in 1:n_node-1
        for j in i+1:n_node
			if x[i,j]!=0
				o=list_node[i,1]
				p=list_node[i,2]
				k=list_node[j,1]
				l=list_node[j,2]

				if (o==k)
					if p>l
						temp=p
						p=l
						l=temp
					end
					for j1 in p:l-1
						liasons[o,j1,o,j1+1]=x[i,j]
					end
				end
				if (p==l)
					if o>k
						temp=o
						o=k
						k=temp
					end
					for i1 in o:k-1
						liasons[i1,p,i1+1,p]=x[i,j]
					end
				end
			end
		end
	end


    println(fout," solution = [")
    for l in 1:n
        s = ""
        for c in 1:m
            if t[l,c]!=0
				value=t[l,c]
                s=s*"$value"
            else
                s=s*" "
            end
			if c<m
				if liasons[l,c,l,c+1]==2
					s=s*" = "
				end
				if liasons[l,c,l,c+1]==1
					s=s*" - "
				end
				if liasons[l,c,l,c+1]==0
					s=s*"   "
				end
			end
        end
        print(fout,'"')
        print(fout, s)
        print(fout,'"')
		if l<n
        	println(fout,",")
		else
			println(fout,"]")
		end
        s = ""
		if l<n
        	for c in 1:n
				if liasons[l,c,l+1,c]==2
					s=s*"||  "
				end
				if liasons[l,c,l+1,c]==1
					s=s*"|   "
				end
				if liasons[l,c,l+1,c]==0
					s=s*"    "
				end
        	end
			print(fout,'"')
	        print(fout, s)
	        print(fout,'"')
	        println(fout,",")
        end

    end

end
function saveInstance(t::Array{Int64, 2}, outputFile::String)

    n = size(t, 1)

    # Open the output file
    writer = open(outputFile, "w")

    # For each cell (l, c) of the grid
    for l in 1:n
        for c in 1:n

            # Write its value
            if t[l, c] == 0
                print(writer, " ")
            else
                print(writer, t[l, c])
            end

            if c != n
                print(writer, ",")
            else
                println(writer, "")
            end
        end
    end

    close(writer)

end



function NBcase(t::Array{Int64}, i::Int64, j::Int64)
	w=0
	w=adjacent(t,i,j)
	l=0

	n_node=0
    for i in 1:n
        for j in 1:m
            if 1<=t[i,j]<=8
                n_node=n_node+1
            end
        end
    end

	list_node=zeros(Int64, node_size, 2)
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

	if w==1
		for p in list_node[i,2]+1:list_node[j,2]-1
			l=l+1
		end
	elseif w==2
		for p in list_node[j,2]+1:list_node[i,2]-1
			l=l+1
		end
	elseif w==3
		for p in list_node[j,1]+1:list_node[i,1]-1
			l=l+1
		end
	elseif w==4
		for p in list_node[i,1]+1:list_node[j,1]-1
			l=l+1
		end
	end
	return l
end



function remplir(t::Array{Int64},i::Int64, j::Int64, x::Array{Float64, 2}, y::Array{Float64, 2})
	w=0
	w=adjacent(t,i,j)
	if (w!=0 && y[i,j]==1)
		l=0
		l=NBcase(t,i,j)
		#j à droite de i
		if w==1
			if x[i,j]==0
				for p in 1:l
					print("   ")
				end
			elseif x[i,j]==1
				for p in 1:l
					print(" - ")
				end
			elseif x[i,j]==2
				for p in 1:l
					print(" = ")
				end
			end
		#j en bas de i
		elseif w==4
			if x[i,j]==0
				for p in 1:l
					println("   ")
				end
			elseif x[i,j]==1
				for p in 1:l
					println(" | ")
				end
			elseif x[i,j]==2
				for p in 1:l
					println(" || ")
				end
			end
		end
	end

end



function resultsArray(outputFile::String)

    resultFolder = "../res/"
    dataFolder = "../data/"

    # Maximal number of files in a subfolder
    maxSize = 0

    # Number of subfolders
    subfolderCount = 0

    # Open the latex output file
    fout = open(outputFile, "w")

    # Print the latex file output
    println(fout, raw"""\documentclass{article}

\usepackage[french]{babel}
\usepackage [utf8] {inputenc} % utf-8 / latin1
\usepackage{multicol}

\setlength{\hoffset}{-18pt}
\setlength{\oddsidemargin}{0pt} % Marge gauche sur pages impaires
\setlength{\evensidemargin}{9pt} % Marge gauche sur pages paires
\setlength{\marginparwidth}{54pt} % Largeur de note dans la marge
\setlength{\textwidth}{481pt} % Largeur de la zone de texte (17cm)
\setlength{\voffset}{-18pt} % Bon pour DOS
\setlength{\marginparsep}{7pt} % Séparation de la marge
\setlength{\topmargin}{0pt} % Pas de marge en haut
\setlength{\headheight}{13pt} % Haut de page
\setlength{\headsep}{10pt} % Entre le haut de page et le texte
\setlength{\footskip}{27pt} % Bas de page + séparation
\setlength{\textheight}{668pt} % Hauteur de la zone de texte (25cm)

\begin{document}""")

    header = raw"""
\begin{center}
\renewcommand{\arraystretch}{1.4}
 \begin{tabular}{l"""

    # Name of the subfolder of the result folder (i.e, the resolution methods used)
    folderName = Array{String, 1}()

    # List of all the instances solved by at least one resolution method
    solvedInstances = Array{String, 1}()

    # For each file in the result folder
    for file in readdir(resultFolder)

        path = resultFolder * file

        # If it is a subfolder
        if isdir(path)

            # Add its name to the folder list
            folderName = vcat(folderName, file)

            subfolderCount += 1
            folderSize = size(readdir(path), 1)

            # Add all its files in the solvedInstances array
            for file2 in filter(x->occursin(".txt", x), readdir(path))
                solvedInstances = vcat(solvedInstances, file2)
            end

            if maxSize < folderSize
                maxSize = folderSize
            end
        end
    end

    # Only keep one string for each instance solved
    unique(solvedInstances)

    # For each resolution method, add two columns in the array
    for folder in folderName
        header *= "rr"
    end

    header *= "}\n\t\\hline\n"

    # Create the header line which contains the methods name
    for folder in folderName
        header *= " & \\multicolumn{2}{c}{\\textbf{" * folder * "}}"
    end

    header *= "\\\\\n\\textbf{Instance} "

    # Create the second header line with the content of the result columns
    for folder in folderName
        header *= " & \\textbf{Temps (s)} & \\textbf{Optimal ?} "
    end

    header *= "\\\\\\hline\n"

    footer = raw"""\hline\end{tabular}
\end{center}

"""
    println(fout, header)

    # On each page an array will contain at most maxInstancePerPage lines with results
    maxInstancePerPage = 30
    id = 1

    # For each solved files
    for solvedInstance in solvedInstances

        # If we do not start a new array on a new page
        if rem(id, maxInstancePerPage) == 0
            println(fout, footer, "\\newpage")
            println(fout, header)
        end

        # Replace the potential underscores '_' in file names
        print(fout, replace(solvedInstance, "_" => "\\_"))

        # For each resolution method
        for method in folderName

            path = resultFolder * method * "/" * solvedInstance

            # If the instance has been solved by this method
            if isfile(path)

                include(path)

                println(fout, " & ", round(solveTime, digits=2), " & ")

                if isOptimal
                    println(fout, "\$\\times\$")
                end

            # If the instance has not been solved by this method
            else
                println(fout, " & - & - ")
            end
        end

        println(fout, "\\\\")

        id += 1
    end

    # Print the end of the latex file
    println(fout, footer)

    println(fout, "\\end{document}")

    close(fout)

end


function performanceDiagram(outputFile::String)

    resultFolder = "../res/"

    # Maximal number of files in a subfolder
    maxSize = 0

    # Number of subfolders
    subfolderCount = 0

    folderName = Array{String, 1}()

    # For each file in the result folder
    for file in readdir(resultFolder)

        path = resultFolder * file

        # If it is a subfolder
        if isdir(path)

            folderName = vcat(folderName, file)

            subfolderCount += 1
            folderSize = size(readdir(path), 1)

            if maxSize < folderSize
                maxSize = folderSize
            end
        end
    end

    # Array that will contain the resolution times (one line for each subfolder)
    results = Array{Float64}(undef, subfolderCount, maxSize)

    for i in 1:subfolderCount
        for j in 1:maxSize
            results[i, j] = Inf
        end
    end

    folderCount = 0
    maxSolveTime = 0

    # For each subfolder
    for file in readdir(resultFolder)

        path = resultFolder * file

        if isdir(path)

            folderCount += 1
            fileCount = 0

            # For each text file in the subfolder
            for resultFile in filter(x->occursin(".txt", x), readdir(path))

                fileCount += 1
                include(path * "/" * resultFile)

                if isOptimal
                    results[folderCount, fileCount] = solveTime

                    if solveTime > maxSolveTime
                        maxSolveTime = solveTime
                    end
                end
            end
        end
    end

    # Sort each row increasingly
    results = sort(results, dims=2)

    println("Max solve time: ", maxSolveTime)

    # For each line to plot
    for dim in 1: size(results, 1)

        x = Array{Float64, 1}()
        y = Array{Float64, 1}()

        # x coordinate of the previous inflexion point
        previousX = 0
        previousY = 0

        append!(x, previousX)
        append!(y, previousY)

        # Current position in the line
        currentId = 1

        # While the end of the line is not reached
        while currentId != size(results, 2) && results[dim, currentId] != Inf

            # Number of elements which have the value previousX
            identicalValues = 1

             # While the value is the same
            while results[dim, currentId] == previousX && currentId <= size(results, 2)
                currentId += 1
                identicalValues += 1
            end

            # Add the proper points
            append!(x, previousX)
            append!(y, currentId - 1)

            if results[dim, currentId] != Inf
                append!(x, results[dim, currentId])
                append!(y, currentId - 1)
            end

            previousX = results[dim, currentId]
            previousY = currentId - 1

        end

        append!(x, maxSolveTime)
        append!(y, currentId - 1)

        # If it is the first subfolder
        if dim == 1

            # Draw a new plot
            plot(x, y, label = folderName[dim], legend = :bottomright, xaxis = "Time (s)", yaxis = "Solved instances",linewidth=3)

        # Otherwise
        else
            # Add the new curve to the created plot
            savefig(plot!(x, y, label = folderName[dim], linewidth=3), outputFile)
        end
    end
end
