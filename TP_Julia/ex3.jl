# 1
vector = Array{Int64}(undef, 0)
matrix = Array{Int64}(undef, 0, 2)
#3
v = [1,2,3,4]
println(v)
m=[1 2;3 4]
println("m = ",m)
println("m[1][1] =", m[1][1])
#4
append!(vector,1)
println("sizeOf(vector) = ", size(vector))