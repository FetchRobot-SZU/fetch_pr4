#ifndef _PARAMETERS_H
#define _PARAMETERS_H
#define NPN 64
#define NMESH  100
#define DM  ((float) (1.0/(NMESH-1.0)))
#define NPIX  1024
#define SCALE 4.0
#define M_PI 3.1415926
#define REALWINSIZE NPIX


#define SCALE 3.


#define M_PI 3.14159265358979323846264338327950288       ////Move to the datastructure.h file 06/23/05

#define SELECTBUFFERSIZE 128

////////////////////////////////////////////////////////
////Load name for object selection
#define NAMEOFSINGELEM      1           ////name of singular element for mouse selection
#define NAMEOFREGELEM       2001        ////name of regular element for mouse selection
#define NAMEOFSINGCONTROL   3000        ////name of singular element control points for mouse selection
#define NAMEOFREGCONTROL    4000        ////name of regular element control points for mouse selection
#define NAMEOFSINGULARITY   5000        ////name of singularities for mouse selection of topology editing
#define NAMEOFLIMITCYCLES   7000        ////name of limit cycles for limit cycle editing
#define NAMEOFSHAPECONTROL  8000        ////name of shape control point for limit cycle shape controlling
#define NAMEOFBRUSHES       10001       ////name of the control points on brushes (or sketches)
#define NAMEOFINTERSECTS    11001       ////name of the intersections of the streets 
#define NAMEOFSEEDS         20001       ////name of the initial seeds
#define NAMEOFMAJROADS      26001       ////name of the major roads
#define NAMEOFTRIANGLE      30001       ////name of triangles for mouse selection

#define SCALARMAX  40
#define SCALARMIN  -40

//
const double DistanceThreshold = 1e-5;
const double EDITBOXSIZE = 0.04;

#define LOWER 1
//

#endif
