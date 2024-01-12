function z = getmovingvolt()

persistent k firstrun
 
if isempty(firstrun)

    k = 1;

    firstrun = 1;
end

k = k+1;

w = 250*randn(1,1);

z = k + w;

end

