function smooth_t = smooth_time(ddxtime, dt0, alpha, thresh)

if nargin < 4, thresh = 2*dt0(1); end
if nargin < 3, alpha = 0.1; end

if length(dt0) == 1
  dt0 = repmat(dt0,length(ddxtime)-1,1);
end

jump = dt0(1);

smooth_t = zeros(size(ddxtime));
smooth_t(1) = ddxtime(1);

for i=2:length(ddxtime);
  
  smooth_t(i) = smooth_t(i-1)+dt0(i-1);
  err = ddxtime(i)-smooth_t(i);
  if abs(err) < thresh
    smooth_t(i) = smooth_t(i)+alpha*err;
  else
      erri = round(err/jump)*jump;
      errf = err-erri;
    smooth_t(i) = smooth_t(i)+alpha*errf + erri;
  end
end