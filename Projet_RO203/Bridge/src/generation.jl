include("io.jl")


function generateInstance(n::Int64, density::Float64)

    t = zeros(Int64,n,n)
	n_node=ceil.(Int, density*n*n)
	list_node=zeros(Int64, 0, 2)
	bridges =zeros(Int64, n_node, n_node)
	nb=0
	while nb<n_node
		nb=0
		list_node=zeros(Int64, 0, 2)
		bridges =zeros(Int64, n_node, n_node)
		g=zeros(Int64,n,n)
		i= ceil.(Int, n * rand())
		j= ceil.(Int, n * rand())
		if g[i,j]==0
			g[i,j]=1
			nb +=1
			list_node=vcat(list_node,[i j])
		end
   		for m in 2:n_node
		 	flag=true
			triedtimes=0
			while flag && triedtimes<2*n_node
				triedtimes+=1
				list_size=size(list_node,1)
				i = ceil.(Int, list_size * rand())
				l=list_node[i,1]
				r=list_node[i,2]
				if rand()*2>1
					k = ceil.(Int, n * rand())
					if g[k,r]==0 
						direction_possible=true
						if k>l
							for o in l+1:k-1
								if g[o,r]==1
									direction_possible=false
								end
							end
							if direction_possible
								g[k,r]=1
								list_node=vcat(list_node,[k r])
								nb+=1
								bridges[i,nb]=ceil.(Int, 2 * rand())
								flag=false
								for o in l+1:k-1
									g[o,r]=1
								end
							end
						elseif k<l
							for o in k+1:l-1
								if g[o,r]==1
									direction_possible=false
								end
							end
							if direction_possible
								g[k,r]=1
								list_node=vcat(list_node,[k r])
								nb+=1
								bridges[i,nb]=ceil.(Int, 2 * rand())
								flag=false
								for o in k+1:l-1
									g[o,r]=1
								end
							end
						end
					end
				else
					k = ceil.(Int, n * rand())
					if g[l,k]==0
						direction_possible=true
						if k>r
							for o in r+1:k-1
								if g[l,o]==1
									direction_possible=false
								end
							end
							if direction_possible
								g[l,k]=1
								list_node=vcat(list_node,[l k])
								nb+=1
								bridges[i,nb]=ceil.(Int, 2 * rand())
								flag=false
								for o in r+1:k-1
									g[l,o]=1
								end
							end
						elseif k<r 
							for o in k+1:r-1
								if g[l,o]==1
									direction_possible=false
								end
							end
							if direction_possible
								g[l,k]=1
								list_node=vcat(list_node,[l k])
								nb+=1
								bridges[i,nb]=ceil.(Int, 2 * rand())
								flag=false
								for o in k+1:r-1
									g[l,o]=1
								end
							end
						end
					end
				end
			end
			#println(list_node)
			#println(bridges)
			#println(note)
		end
    end

	for i in 1:n_node-1
		for j in i+1:n_node
			t[list_node[i,1],list_node[i,2]]+=bridges[i,j]
			t[list_node[j,1],list_node[j,2]]+=bridges[i,j]
		end
	end

	return t
end

"""
Generate all the instances

Remark: a grid is generated only if the corresponding output file does not already exist
"""
function generateDataSet()

    # For each grid size considered
    for size in [4, 7, 12]

		for density in [0.3,0.5,0.6]
            	# Generate 10 instances
            	for instance in 1:10

                	fileName = "../data/instance_t" * string(size) * "_d" * string(density) * "_" * string(instance) * ".txt"

                	if !isfile(fileName)
                    	println("-- Generating file " * fileName)
                    	saveInstance(generateInstance(size,density), fileName)
                	end
            	end

        end
    end
end
