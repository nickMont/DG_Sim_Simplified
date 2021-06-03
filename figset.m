% figset
% Set font size of figures for printing

fs = 12;
fn = 'times new roman';
%fn = 'helvetica';
%fn = 'courier new';

set(gca, 'fontsize', fs, 'fontname', fn);
hy = get(gca, 'ylabel');
hx = get(gca, 'xlabel');
set(hy, 'fontsize', fs, 'fontname', fn);
set(hx, 'fontsize', fs, 'fontname', fn);
title('');
