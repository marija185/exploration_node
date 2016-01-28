close all

boja='b'
staripoligon
title('stari poligon iz proslog koraka')

boja='b'
novipoligon
boja='r--'
lokpolygon
crtajnum
title('poligon iz trenutnog ocitanja')

boja='b'
poligon
boja='r--'
unija
crtaj
title('poligon nakon unije')

figure
boja='k'
prijeunija
crtaj
title('stari poligon i poligon iz trenutnog ocitanja nalijepljeni')

figure
boja='k'
posunija
crtaj
title('treba biti isto uniji')
