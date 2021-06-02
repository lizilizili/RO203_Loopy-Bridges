
function print(a::String)
	println(a)
end

function square(n::Int64)
	return n*n
end


function incDec(a::Int64,b::Int64)
	return a-b,a+b
end

a=Int64(3)
println("a=",a)

function eq(a::Int64,b::Int64)
	if a==b
		println(a," = ",b)
	else
		println(a," != ",b)
	end
end

function fibonacci()
	a = Int64(0)
	b = Int64(1)
	c = Int64(0)
	println("fi(0) = ",a)
	println("fi(1) = ",b)
	count=Int64(2)
	while c < 1000000
		c=a+b
		println("fi(",count,") = ",c)
		a=b
		b=c
		count=count+1
	end
end

function fractions(a::Int64)
	for i in 1:a
		println(1/i)
	end
end

function factorial(a::Int64)
	s=1
	for i in 2:a
		s = s * a
	end
	return s
end