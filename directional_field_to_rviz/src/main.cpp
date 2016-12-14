#include <GL/glut.h>
#include <stdlib.h>
#include <math.h>
#include "directional_field_to_rviz/GlView.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
//#include "directional_field_to_rviz/TexturedQuad.h"
//#include "directional_field_to_rviz/TexturedQuadArray.h"

GlView *zlt_view;
static int du=90,oldmy=-1,oldmx=-1;
image_transport::Publisher rgb_pub;
//float realWorld_to_field_scale=10;
//float realWorld_to_field_offset_x=0.5;
//float realWorld_to_field_offset_y=0.5;

void publish_image(){
    unsigned char  *rgb_im = (unsigned char *)malloc(NPIX * NPIX*3* sizeof(unsigned char));
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0,NPIX,NPIX, GL_RGB, GL_UNSIGNED_BYTE, rgb_im);
    cv_bridge::CvImage img_ptr_rgb;//=new cv_bridge::CvImage ;
    img_ptr_rgb.image=cv::Mat(NPIX,NPIX, CV_8UC3);
    cv::Mat& mat_rgb = img_ptr_rgb.image;

    cv::MatIterator_<cv::Vec3b> it=mat_rgb.begin<cv::Vec3b>();
    for (int i = NPIX-1;i >-1; i--)
    {
        for (int j = 0; j <  NPIX; j++)
        {
            (*it)[2]=rgb_im[i * NPIX*3 + j*3+0];  (*it)[1]=rgb_im[i * NPIX*3 + j*3+1]; (*it)[0]=rgb_im[i * NPIX*3 + j*3+2];
            ++it;
        }
    }
    sensor_msgs::ImagePtr msg_rgb=cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_rgb).toImageMsg();
    rgb_pub.publish(msg_rgb);
}

void display(){
    ros::spinOnce();
    zlt_view->DrawGLScene(GL_RENDER);
    publish_image();

}

void mouseClick(int btn, int state, int x, int y)
{
    if(btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
        zlt_view->OnLButtonDown(x,y);
    }
    if(btn == GLUT_LEFT_BUTTON && state == GLUT_UP){
        zlt_view->OnLButtonUp(x,y);
    }
}

void keyboard(unsigned char key, int x, int y)  
{  
	switch (key) {  
	case 'l':  
        zlt_view->listen_to_robot_loc();
        //lt_view.test();
        glutPostRedisplay();
		break;   
	case 's':
        zlt_view->cancelPath();
        glutPostRedisplay();
		break;
    case 't':
        zlt_view->setObstacleAndTarget();
        glutPostRedisplay();
        break;
    case 'm':
        zlt_view->drawDirMap();
        glutPostRedisplay();
        break;
    case 'y':
        zlt_view->setDegenerateElem(0.5,0.5,2);
        glutPostRedisplay();
        break;
    case 'g':
        zlt_view->reset_major_path();
        zlt_view->listen_to_robot_loc();
        zlt_view->gen_major_path(false);
        glutPostRedisplay();
        break;
	case 27:  
		exit(0);  
		break;  
	}  
}  
 
void mouseMove(int x,int y) //
{
    zlt_view->OnMouseMove(x,y);
}
/*----------------------------------------------------*/
int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    zlt_view=new GlView(nh);
    image_transport::ImageTransport it(nh);
    rgb_pub = it.advertise("/zlt_haha", 1);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB); 
    glutInitWindowSize(NPIX, NPIX);
	glutCreateWindow("Direction_filed");
    zlt_view->GlviewInit();
	glutDisplayFunc(display);
	glutIdleFunc(display);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMove);
	glutKeyboardFunc(keyboard);
     zlt_view->makePatterns();
	glutMainLoop();
	return 0;
}
