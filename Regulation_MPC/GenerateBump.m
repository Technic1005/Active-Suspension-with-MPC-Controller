function [zf,zr,dotzf,dotzr] = GenerateBump(L, A, Wheelbase, V, t0, ts, t)

T=0:ts:t;

zf = zeros(1, length(T));
dotzf = zeros(1, length(T));
zr = zeros(1, length(T));
dotzr = zeros(1, length(T));

for i =1:length(T)
    t = T(i);
    if t>=t0 && t<=t0+L/V
        zf(i) = A/2*(1-cos(2*pi*V/L*(t-t0)));
        dotzf(i) = A/2*sin(2*pi*V/L*(t-t0))*2*pi*V/L;
    end
    if t>=t0+Wheelbase/V && t <= t0+(L+Wheelbase)/V
        zr(i) = A/2*(1-cos(2*pi*V/L*(t-t0-Wheelbase/V)));
        dotzr(i) = A/2*sin(2*pi*V/L*(t-t0-Wheelbase/V))*2*pi*V/L;
    end
end
end
% figure()
% hold on
% plot(T, dotzf)
% plot(T, dotzr)