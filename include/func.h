
#include "pravac.h"
#include "cudo.h"
#include "gpc2.h"  

void gpc_add_DuzinaPolygon(gpc_polygon *p, vector<Duzina> & polygon);
bool pointInPolygon(gpc_polygon *polygon, Tocka T);
