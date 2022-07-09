clear all
clc
close all

T_cycle = 0.5;
t_vec=linspace(0,2*T_cycle,500);


Lstep = .05  ;
FootHeight = 0 ;
t1 = T_cycle/4  ;
t2 = T_cycle/2 ;
t3 = 3*T_cycle/4  ;
t4 = T_cycle ;

for i=1:length(t_vec)
    t_shit=t_vec(i);
    t = mod(t_shit,t4);
    tl = mod(t_shit-0.5*T_cycle,T_cycle);
    
    c = floor(t_shit/T_cycle);
    d = floor( (t_shit+0.5*T_cycle )/T_cycle);
    if mod(t,t4) < t1
        xr(i) = -Lstep + c*2*Lstep;
        zr(i) = FootHeight;
        xl(i) = -1.5144*tl^3 + 1.5855*tl^2 - 0.2334*tl - 0.0404 + d*2*Lstep;
        zl(i) = 8.1169*tl^3 - 9.8338*tl^2 + 3.575*tl - 0.34347;
    elseif mod(t,t4)<t2
        xr(i) = -1.2848*t^3 + 1.3202*t^2 - 0.1938*t - 0.0435 + c*2*Lstep;
        zr(i) = -19.724*t^3 + 10.2116*t^2 - 1.2359*t + 0.0412;
        xl(i) = Lstep + d*2*Lstep;
        zl(i) = FootHeight;
    elseif mod(t,t4)<t3
        xr(i) = -1.5144*t^3 + 1.5855*t^2 - 0.2334*t - 0.0404 + c*2*Lstep;
        zr(i) = 8.1169*t^3 - 9.8338*t^2 + 3.575*t - 0.34347;
        xl(i) = -Lstep + d*2*Lstep;
        zl(i) = FootHeight;
    else
        
        xr(i) = Lstep + c*2*Lstep;
        zr(i) = FootHeight;
        xl(i) = -1.5144*tl^3 + 1.5855*tl^2 - 0.2334*tl - 0.0404 + d*2*Lstep;
        zl(i) = -19.724*tl^3 + 10.2116*tl^2 - 1.2359*tl + 0.0412;
    end
    
end
plot(t_vec,xr)
hold on
plot(t_vec,zr)
plot(t_vec,xl)
hold on

plot(t_vec,zl)
legend('xr','zr','xl','zl')
xlabel('Time (s)')
ylabel('Distance (m)')
set(gca,'FontSize',14)

title('MATLAB FTW','FontSize',20)
% legend('xl','zr')
