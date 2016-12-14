#ifndef _GLVIEW_H
#define _GLVIEW_H

#include <GL/glut.h> 
#include "parameters.h"
#include "dataStructure.h"
#include "icVector.h"
#include "QuadMesh.h"
#include "util.h"
#include <std_msgs/String.h>
#include "EvenStreamlinePlace.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "octomap_fetch/Obstacle.h"
#include "octomap_fetch/Map.h"
#include "octomap_fetch/Target.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>

// CGlView

class GlView
{
public:
        GlView(ros::NodeHandle &nh);
    ~GlView();

	// Attributes
public:
	//
	double  ten_tmax ;
	double  ten_dmax ;
	int    iframe ; 
	int    Npat  ;
	int    alpha  ;

	float  tmax ;
	float  dmax ;
        //ros
        ros::NodeHandle nh_;
        tf::TransformListener listener;
        nav_msgs::Path path;
        ros::Publisher pub_plan;
        double realWorld_to_field_scale;
        double realWorld_to_field_offset_x;
        double realWorld_to_field_offset_y;
        ros::Publisher cancel_pub;
        ros::Subscriber obstacle_sub;
        ros::Subscriber dirMap_sub;
        ros::Subscriber target_sub;
        octomap_fetch::Target cur_target;
        octomap_fetch::Obstacle obstacle;
        octomap_fetch::Map dirMap;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<std::vector<cv::Point> > m_hull;
        std::vector<std::vector<cv::Point2f> > dirMap2World;
        std::vector<std::vector<cv::Point2f> > contours2World;
        std::vector<cv::Vec4i> hierarchy;
        icVector2 m_robotDirect;
	//
	double trans_x,trans_y,zoom_factor;
	float inverse_tran[16];
	int MaxNumSingularElems;                  //Maximum number of singular elements
	int MaxNumRegularElems;                   //Maximum number of regular elements
	int MaxNumSingularities;
	int curMaxNumTenRegElems;
	int MaxNumDegeneratePts;
	int major_iframe;
	int minor_iframe;
	int ntenelems;
	int nten_regelems;
	int ndegpts;
	DegeneratePt *degpts;
	TenRegularElem *ten_regularelems;
	GLuint tentextnames[16];
	GLubyte major_tex1[NPIX][NPIX][3], major_tex2[NPIX][NPIX][3], major_tex[NPIX][NPIX][3],
		minor_tex1[NPIX][NPIX][3], minor_tex2[NPIX][NPIX][3], minor_tex[NPIX][NPIX][3],
		major_alpha_map[NPIX][NPIX][3], minor_alpha_map[NPIX][NPIX][3];

	GLubyte major_temp[NPIX][NPIX][4], minor_temp[NPIX][NPIX][4];

	/*the quad mesh object for the tensor field design*/
	QuadMesh *quadmesh;
	//////////////////////////////////////////////////////////////////////////
	SingularElement *singularelem;          //Singular elememts' list
	int cur_singelem_index;

	RegularElement *regularelem;            //regular elememts' list
	int cur_regelem_index;
	int prev_num_reg_elem;                  //for shape design generated limit cycle

	Singularities *singularities;           //being captured singularites' list
	int cur_singularity_index;
        //////////////////////////////////////////////////////////////////////////
	bool showIBFVOn;
	bool showRegularElemOn;
	bool showGridOn;
	bool showSingularitiesOn;
        Seed *robot_loc;
	//////////////////////////////////////////////////////////////////////////
	icMatrix2x2 *pre_tenfield;
	bool m_LeftButtonDown;
	int cur_chosen_region;
	int  m_LeftButtonDown_posX;
	int  m_LeftButtonDown_posY;
	double pan_s_old, pan_t_old; 
	double s_old, t_old;
	Degenerate_Design *ten_designelems;
	int curMaxNumTenDesignElems;
	bool please_comb_prefield;
	unsigned char cur_max_reg_index ;  /*   current maximum region index   */
	Polygon3D Object;
	int ndegenerate_tris;
	int *degenerate_tris;
	bool showMajorTenLine;
	EvenStreamlinePlace *major_path;
	//////////////////////////////////////////////////////////////////////////
	// Operations for OpenGl window setting and displaying
public:
	void GlviewInit();
	void InitVFVariables();
	//////////////////////////////////////////////////////////////////////////
	int InitGL();	           //initialize the OpenGl envrionment
	void init_texture_objs();
	void tensor_init_tex();
	void init_degpts();
	void init_regular_ten_designelems();
    void init_scalar_singular_elemlist();
	//////////////////////////////////////////////////////////////////////////
	int DrawGLScene(GLenum mode);       //The same function as display routine to show the visual effect in the opengl window
	void cal_inverse_transform();
	void transform_fun();
	void HsvRgb( float hsv[3], float rgb[3] );
	int without_anti_aliasing(GLenum mode);
	void make_tens_Patterns(void);
	void makePatterns(void); 
	//////////////////////////////////////////////////////////////////////////
	void render_majorfield_quad();
	void render_a_map(unsigned char *map);
	void major_vis_quad();
	void render_ibfv_tens_quad(bool major_minor, bool x_y);
	void render_tensor_blend();
	void render_tensor_final(unsigned char majororminor);
	void drawSolidRect_size(double cx, double cy, double R);
	void drawSolidCircle_size(double cx, double cy, double R);
	void display_tenRegElem(GLenum mode);
	void display_design_grid();
	void display_degenerate_pts(GLenum mode);
	void draw_hollow_circle_size(double cx, double cy, double R);

//////////////////////////////////////////////////////////////////////////
	void save_cur_field();
    void ScreenToWorld(double position[3], int x,int y);
	void OnLButtonDown(int x, int y);
	void OnLButtonUp(int x, int y);
	void OnMouseMove(int x,int y);
	void set_ten_regBasis(double x, double y, int type);
	unsigned char get_region_id(double x, double y);
	void set_ten_regDir(double x, double y);
	void cal_tensorvals_quad_inReg();
	void get_tensor_inReg(double x, double y, double t[4], int regionid, bool inbrush);
	void init_ten_designelems();
	void cal_all_eigenvecs_quad();
	void cal_eigenvecs_onevert_quad(int ver);
	void cal_eigen_vector_sym(icMatrix2x2 m, icVector2 ev[2]);
	void normalized_tensorfield();
	void normalized_tensorfield_quad();
	void render_alpha_map_quad(bool major_minor);
	void locate_degpts_cells_tranvec_quad(void);
	void compute_degpts_pos_tranvec_quad();
	void compute_onedegpt_pos_tranvec_quad(int cellid, double &x, double &y);
	void compute_a_alongx_degptlocate(double &a, icVector2 v0, icVector2 v1, icVector2 v2, icVector2 v3,
		icVector2 &v0v1, icVector2 &v2v3);
	void display_major_tenlines(GLenum mode);
        void gen_major_path(bool type);
        void reset_major_path();
	void test();
        void set_robot_loc(double x,double y);
        void listen_to_robot_loc();
	//////////////////////////////////////////////////////////////////////////
	void setRegularElem(double base_x, double base_y,double end_x,double end_y);
    void setDegenerateElem(double center_x,double center_y,int type);
    void setDriveForce();
    void addto(double x, double y, int triangle, int type);
    void init_tenelem_EditBox(int index, double x, double y);
    int get_cellID_givencoords(double x, double y);

	//////////////////////////////////////////////////////////////////////////
        //ros
      void addRobotWayPoint();
      void cancelPath();
      void obstacleCallback(const octomap_fetch::Obstacle &msg);
      void targetCallback(const octomap_fetch::Target &msg);
      void dirMapCallback(const octomap_fetch::Map &msg);
      void setObstacleAndTarget();
      void drawDirMap();
      void getObstacleContour();
      void resetRegularAndDegenrateElem();
        //
};



#endif
