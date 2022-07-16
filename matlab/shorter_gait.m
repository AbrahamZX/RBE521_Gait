clear all
clc
close all

T_cycle = 1; % works for xr, zr
t_vec=linspace(0,5*T_cycle,1001);


Lstep = .05  ;
% Lstep = 0.2;

FootHeight = 0 ;

t2 = 0.10;
t3 = 0.25;
t4 = 0.5;

for i=1:length(t_vec)
    t_shit=t_vec(i);
    t = mod(t_shit,T_cycle);
    tl = mod(t_shit-0.5*T_cycle,T_cycle);
    
    c = floor(t_shit/T_cycle);
%     c=0;
    d = floor( (t_shit+0.5*T_cycle )/T_cycle);
    if mod(t,T_cycle) < t2
        xr(i) = -Lstep + c*2*Lstep;
        zr(i) = FootHeight;
        
        
    elseif mod(t,T_cycle)<t3
        xr(i) = -1.2848*t^3 + 1.4202*t^2 - 0.1938*t - 0.0435 + c*2*Lstep;
        zr(i) = -19.724*t^3 + 10.2116*t^2 - 1.2359*t + 0.0412;
    elseif mod(t,T_cycle)<t4
        xr(i) = -1.5144*t^3 + 1.5855*t^2 - 0.2334*t - 0.0404 + c*2*Lstep;
        zr(i) = 8.1169*t^3 - 9.8338*t^2 + 3.575*t - 0.3437;
        
    else
        
        xr(i) = Lstep + c*2*Lstep;
        zr(i) = FootHeight;
    end
    
%     if mod(tl,T_cycle)<t2
%         xl(i) = -1.5144*tl^3 + 1.5855*tl^2 - 0.2334*tl - 0.0404 + d*2*Lstep;
%         zl(i) = 8.1169*tl^3 - 9.8338*tl^2 + 3.575*tl - 0.3437;
%         
%     elseif mod(tl,T_cycle)<t3
%         xl(i) = Lstep + d*2*Lstep;
%         zl(i) = FootHeight;
%     elseif mod(tl,T_cycle)<t4
%         xl(i) = -Lstep + d*2*Lstep;
%         zl(i) = FootHeight;
%     else
%         xl(i) = -1.2848*tl^3 + 1.4202*tl^2 - 0.1938*tl - 0.0435 + d*2*Lstep;
%         zl(i) = -19.724*tl^3 + 10.2116*tl^2 - 1.2359*tl + 0.0412;
%         
%         
%     end
end

xl = xr;
zl = zr;
xr = xr+Lstep;
t_vec_l = t_vec - (T_cycle/2);

t_vec_new = t_vec_l(101:end);
xl_new = xl(101:end);
zl_new = zl(101:end);

xr_new = xr(1:end-100);
zr_new = zr(1:end-100);

figure
plot(t_vec_new,xr_new,'LineWidth',2)
hold on
plot(t_vec_new,zr_new,'LineWidth',2)
plot(t_vec_new,xl_new,'LineWidth',2)
plot(t_vec_new,zl_new,'LineWidth',2)
legend('X_r','Z_r','X_l','Z_l')
xlabel('Time (sec)')
ylabel('Displacement (m)')
set(gca,'FontSize',14)
axis([0 4.5 0  0.5])
grid on

% data_to_export = [t_vec_new;xr_new;zr_new;xl_new;zl_new];
% 
% csvwrite('data.csv',data_to_export);
