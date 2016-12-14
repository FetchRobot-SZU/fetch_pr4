#ifndef _UTIL_H
#define _UTIL_H
void transform_point3D(float p[3], float rot_mat[16]);
double bilinear_interpolate(double a, double b, double f00, double f01, double f10, double f11);
bool is_repeated_elem(int *a, int b, int num);
/* meaning of return value
 0----Intersection dosn't exists                                                   
 1----Intersection exists.                                                        
 2----two line segments are parallel.                                         
 3----two line segments are collinear, but not overlap.                      
 4----two line segments are collinear, and share one same end point.       
 5----two line segments are collinear, and overlap.                           
*/    
inline double fabs (double x){return x>0?x:-x; }
inline double max (double x,double y){return x>y?x:y; }
inline double min (double x,double y){return x>y?y:x; }
int GetIntersection2(double PointA[2], double PointB[2], double PointC[2], double PointD[2], double t[2]);
#endif
