function x = time_reg(ta, va, tb, vb)


dt_a = (ta(end)-ta(1))/(length(ta)-1);
dt_b = (tb(end)-tb(1))/(length(tb)-1);
dt_s = min(dt_a,dt_b);

vas = interp1(ta,va,ta(1):dt_s:ta(end),'linear',0);
vbs = interp1(tb,vb,tb(1):dt_s:tb(end),'linear',0);
c = conv(vas,fliplr(vbs));
[foo,i] = max(c);
delta_t = -(i-length(vbs))*dt_s;
drift = 0.000;

x0 = [delta_t drift];

%keyboard
if false
    winN = 1024;
    shift = 128;
    w = cos(linspace(-pi/2,pi/2,winN+2));
    w = w(2:end-1); w = w/sum(w);
    i = 1:shift:length(vas)-winN;

    c = zeros(winN-1+length(vbs),length(i));
    for k=1:length(i)
      mask = fliplr(w.*vas(i(k)-1+(1:winN)));
      %mask = mask-mean(mask); 
      %mask = mask/std(mask);
      ci = conv(vbs,mask);
      c(:,k) = ci(mod((1:end)+i(k)+length(mask),length(ci)-1)+1);
    end
    imagesc((i)*dt_s,(1:size(c,1))*dt_s,sqrt(c));

    keyboard


    dx = linspace(-.1,.1,200);
    dy = linspace(-.002,.002,200);

    E = zeros(length(dy),length(dx));
    for j=1:length(dx)
      j
      for i=1:length(dy);
        E(i,j) = time_reg_err(x0+[dx(j) dy(i)],ta,va,tb,vb);
      end
      if mod(j,5)==1
        imagesc(dx(1:j),dy,E(:,1:j)), colorbar
        drawnow
      end
    end
    keyboard
end


lb = []; ub = [];
opts = optimset('MaxIter',500,'MaxFunEvals',500, ...
    'TolFun', 1e-8, 'TolX', 1e-8);%,'Display','iter','PlotFcns',@optimplotfval);


x = fminsearch(@(x) time_reg_err(x,ta,va,tb,vb), x0, opts);

%x = lsqnonlin(@(x) time_reg_err(x,ta,va,tb,vb), x0, lb,ub,opts);

function err = time_reg_err(x,ta,va,tb,vb)

ta_b = ta*(1+x(2)) + x(1);
vb_a = interp1(tb,vb,ta_b,'linear',0);

err = -sum(va.*vb_a);
%err = va - vb_a;

%plot(x(1),x(2),'.');
