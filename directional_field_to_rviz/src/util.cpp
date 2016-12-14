#include "directional_field_to_rviz/util.h"

void transform_point3D(float p[3], float rot_mat[16])
{
	double tmp[3] = {0.};

	tmp[0] = rot_mat[0] * p[0] + rot_mat[4] * p[1] + rot_mat[8]  * p[2] + rot_mat[12];
	tmp[1] = rot_mat[1] * p[0] + rot_mat[5] * p[1] + rot_mat[9]  * p[2] + rot_mat[13];
	tmp[2] = rot_mat[2] * p[0] + rot_mat[6] * p[1] + rot_mat[10] * p[2] + rot_mat[14];

	p[0] = tmp[0];
	p[1] = tmp[1];
	p[2] = tmp[2];
}

double bilinear_interpolate(double a, double b, 
									 double f00, double f01, double f10, double f11)
{
	return (f00*(1-a)*(1-b)+f10*a*(1-b)+f01*(1-a)*b+f11*a*b);
}
bool is_repeated_elem(int *a, int b, int num)
{
	int i;
	for(i = 0; i < num; i++)
	{
		if(a[i] == b)
			return true;
	}
	return false;
}
/* meaning of return value
 0----Intersection dosn't exists                                                   
 1----Intersection exists.                                                        
 2----two line segments are parallel.                                         
 3----two line segments are collinear, but not overlap.                      
 4----two line segments are collinear, and share one same end point.       
 5----two line segments are collinear, and overlap.                           
*/    

int GetIntersection2(double PointA[2], double PointB[2], double PointC[2], double PointD[2], double t[2])
{

    double delta;
    double t1,t2;
    double a,b,c,d;
    double xba,yba,xdc,ydc,xca,yca;

    xba=PointB[0]-PointA[0];    yba=PointB[1]-PointA[1];
    xdc=PointD[0]-PointC[0];    ydc=PointD[1]-PointC[1];
    xca=PointC[0]-PointA[0];    yca=PointC[1]-PointA[1];

    delta=xba*ydc-yba*xdc;
    t1=xca*ydc-yca*xdc;
    t2=xca*yba-yca*xba;

    if(delta!=0)
    {
        t[0]=t1/delta;   t[1]=t2/delta;
        /*two segments intersect (including intersect at end points)*/
        //if ( t[0]<=1 && t[0]>=0 && t[1]<=1 && t[1]>=0 ) return 1;
        if ( t[0]<=1 && (t[0]>=0 || fabs (t[0])<=1.e-8)
			&& t[1]<=1 && (t[1]>=0|| fabs (t[1])<=1.e-8)) //set threshold to allow some numerical errors
			return 1;
        else return 0; 
    }

    else
    {       
        /* AB & CD are parallel. */
        if ( (t1!=0) && (t2!=0) ) return 2;

        /* when AB & CD are collinear */

        /*if AB isn't a vertical line segment, project to x-axis */
        if(PointA[0]!=PointB[0])   
        {
            a=min(PointA[0],PointB[0]); b=max(PointA[0],PointB[0]);
            c=min(PointC[0],PointD[0]); d=max(PointC[0],PointD[0]);

            if ( (d<a) || (c>b) ) return  3;
            else if( (d==a) || (c==b) ) return 4;  
            else return 5;
        }

        else         /* if AB is a vertical line segment, project to y-axis */  
        {

            a=min(PointA[1],PointB[1]); b=max(PointA[1],PointB[1]);
            c=min(PointC[1],PointD[1]); d=max(PointC[1],PointD[1]); 

            if( (d<a) || (c>b) ) return  3;
            else if( (d==a) || (c==b) ) return 4;
            else return 5;
        }
    }
}
