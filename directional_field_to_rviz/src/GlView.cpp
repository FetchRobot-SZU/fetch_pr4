#include"directional_field_to_rviz/GlView.h"

GlView::GlView(ros::NodeHandle &nh):nh_(nh)
{
	showGridOn=false;
	showIBFVOn=true;
    showRegularElemOn=true;
    showSingularitiesOn=false;
	showMajorTenLine=true;
	major_iframe = 0;
	minor_iframe = 0;
	quadmesh = NULL;
	ntenelems = 0;
	nten_regelems = 0;
	ndegpts = 0;
	ten_regularelems=NULL;
	degpts = NULL;
	curMaxNumTenRegElems =100;
	MaxNumDegeneratePts = 200;
	//scalar_singular_elems=NULL;
	//
	ten_tmax   = NPIX/(SCALE*NPN);
    ten_dmax   = SCALE/NPIX;
	iframe = 0; 
	Npat   = 32;
	alpha  = (0.12*255);
	tmax   = NPIX/(SCALE*NPN);
	dmax   = SCALE/NPIX;
	//
	pre_tenfield=NULL;
	m_LeftButtonDown=false;
	cur_chosen_region=0;
	m_LeftButtonDown_posX=0;
	m_LeftButtonDown_posY=0;
	pan_s_old=0;
	pan_t_old=0;
	s_old=0;
	t_old=0;
	ten_designelems = NULL;
	curMaxNumTenDesignElems = 100;
	please_comb_prefield=false;
	cur_max_reg_index = 0;  /*   current maximum region index   */
	ndegenerate_tris = 0;
	degenerate_tris = NULL;
	major_path=NULL;
    //ros
    robot_loc=new Seed();
    robot_loc->pos[0]=0.5;
    robot_loc->pos[1]=0.5;
    robot_loc->triangle=0;
    realWorld_to_field_scale=20;
    realWorld_to_field_offset_x=0.5;
    realWorld_to_field_offset_y=0.5;
    pub_plan = nh_.advertise<nav_msgs::Path>("/reference_path",1);
    cancel_pub = nh_.advertise<std_msgs::String>("/cancel_path", 10);
    obstacle_sub=nh_.subscribe("arrows", 10, &GlView::obstacleCallback, this);
    dirMap_sub=nh_.subscribe("dirMap",10,&GlView::dirMapCallback,this);
    target_sub=nh_.subscribe("target",10,&GlView::targetCallback,this);
    cur_target.target_x=realWorld_to_field_offset_x;
    cur_target.target_y=realWorld_to_field_offset_y;
    //
	//
}
GlView::~GlView(){

}
void GlView::GlviewInit(){
	quadmesh = new QuadMesh(100, 100, -0.01, 1.01, -0.01, 1.01);
	major_path=new EvenStreamlinePlace(false,300);
	major_path->setGlView(this);
	major_path->init();
	major_path->set_default_parameters(false);
	major_path->reset_placement_quad();
	major_path->init_major_minor_line_info(false);
	pre_tenfield=new icMatrix2x2[quadmesh->nverts];
	init_ten_designelems();
	init_degpts();
	zoom_factor = 1;
	trans_x=trans_y=0;
	cal_inverse_transform();
	//////////////////////////////////////////////////////////////////////////dlg above
	InitGL();	

	make_tens_Patterns();
	tensor_init_tex();

}

void GlView::make_tens_Patterns(void)
{
	int lut[256];
	int phase[NPN][NPN];
	GLubyte pat[NPN][NPN][4];
	GLubyte spat[NPN][NPN][4];
	int i, j, k, t;

	for (i = 0; i < 256; i++) lut[i] = i < 127 ? 0 : 255;
	for (i = 0; i < NPN; i++)
		for (j = 0; j < NPN; j++) phase[i][j] = rand() % 256; 

	for (k = 200; k < Npat+200; k++) {
		//t = k*256/Npat;                           //t is used to control the animation of the image
		for (i = 0; i < NPN; i++) 
			for (j = 0; j < NPN; j++) {
				pat[i][j][0] = 
					pat[i][j][1] = 
					pat[i][j][2] = (GLubyte)lut[ phase[i][j] % 255];
				pat[i][j][3] = alpha+5;

				spat[i][j][0] = 
					spat[i][j][1] = 
					spat[i][j][2] = (GLubyte)lut[ phase[i][j] % 255];
				spat[i][j][3] = alpha+5;
			}

			glNewList(k + 1, GL_COMPILE);  //major texture
			glTexImage2D(GL_TEXTURE_2D, 0, 4, NPN, NPN, 0, 
				GL_RGBA, GL_UNSIGNED_BYTE, pat);
			glEndList();


			glNewList(k + 1 + 100, GL_COMPILE);       //This is for minor image
			glTexImage2D(GL_TEXTURE_2D, 0, 4, NPN, NPN, 0, 
				GL_RGBA, GL_UNSIGNED_BYTE, spat);
			glEndList();   
	}
}


void GlView::InitVFVariables()
{
	int i;
	MaxNumSingularElems = 5;                  //Maximum number of singular elements
	MaxNumRegularElems = 1;                  //Maximum number of regular elements
	MaxNumSingularities = 1;                 //Maximum number of being captured singularities

	////singular Elements
	singularelem = (SingularElement*) malloc(sizeof(SingularElement)* MaxNumSingularElems);  //Singular elememts' list
	cur_singelem_index = 0;
	for(i = 0; i < MaxNumSingularElems; i++)
	{
		singularelem[i].ID = -1;
		singularelem[i].type = -1;
	}
	//regular Elements
	regularelem = (RegularElement*) malloc(sizeof(RegularElement) * MaxNumRegularElems);     //regular elememts' list
	cur_regelem_index = 0;
	for(i = 0; i < MaxNumRegularElems; i++)
	{
		regularelem[i].ID = -1;
		regularelem[i].type = -1;
	}

	////Singularities
	singularities = (Singularities*) malloc(sizeof(Singularities) * MaxNumSingularities);    //being captured singularites' list
	cur_singularity_index = 0;

}
int GlView::InitGL()								// All Setup For OpenGL Goes Here
{
	//glViewport(0, 0, (GLsizei) NPIX, (GLsizei) NPIX);
	glViewport(0, 0, (GLsizei)REALWINSIZE, (GLsizei)REALWINSIZE);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity(); 
	gluOrtho2D(0, 1, 0, 1);
	//glTranslatef(-1.0, -1.0, -1.0); 
	//glScalef(2.0, 2.0, 1.0);
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_WRAP_S, GL_REPEAT); 
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_WRAP_T, GL_REPEAT); 
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, 
		GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, 
		GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClear(GL_COLOR_BUFFER_BIT);

	glDisable(GL_STENCIL_TEST);

	return true;										// Initialization Went OK
}
void GlView::init_regular_ten_designelems()
{
	/*initialize regular design element*/
	if(ten_regularelems == NULL)
	{
		ten_regularelems=(TenRegularElem *)malloc(sizeof(TenRegularElem)*curMaxNumTenRegElems);
		if(ten_regularelems == NULL)
			exit(-1);
    }else
	nten_regelems = 0;
}

void GlView::init_degpts()
{
	//curMaxNumDegPts = 200;
	if(degpts == NULL)
	{
		degpts = (DegeneratePt*)malloc(sizeof(DegeneratePt)*MaxNumDegeneratePts);
		if(degpts == NULL)
			exit(-1);
	}
	ndegpts = 0;

	int i;
	for(i=0; i<quadmesh->nfaces; i++)
	{
		quadmesh->quadcells[i]->degpt_index = -1;
	}
}

void GlView::init_texture_objs()
{
	/*generate texture names*/
	glGenTextures(8, tentextnames);

	/*bind the texture maps*/
	glBindTexture(GL_TEXTURE_2D, tentextnames[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, major_tex1);

	glBindTexture(GL_TEXTURE_2D, tentextnames[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, major_tex2);


	glBindTexture(GL_TEXTURE_2D, tentextnames[2]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, major_tex);

	glBindTexture(GL_TEXTURE_2D, tentextnames[3]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, minor_tex1);

	glBindTexture(GL_TEXTURE_2D, tentextnames[4]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, minor_tex2);


	glBindTexture(GL_TEXTURE_2D, tentextnames[5]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGBA, GL_UNSIGNED_BYTE, minor_tex);
}

/*initialize the textures*/
void GlView::tensor_init_tex()
{
	glDrawBuffer(GL_BACK);

	glCallList(201);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,  0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0,  ten_dmax); glVertex2f(0.0, 1.0);
	glTexCoord2f(ten_dmax, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(ten_dmax, ten_dmax); glVertex2f(1.0, 1.0);
	glEnd();

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex1);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex2);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex);


	glCallList(301);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,  0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0,  ten_dmax); glVertex2f(0.0, 1.0);
	glTexCoord2f(ten_dmax, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(ten_dmax, ten_dmax); glVertex2f(1.0, 1.0);
	glEnd();

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, minor_tex1);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, minor_tex2);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, minor_tex);
}


void GlView::cal_inverse_transform()
{
	//wglMakeCurrent(m_hDC, m_hglRC);
	glMatrixMode(GL_MODELVIEW_MATRIX);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(-trans_x, -trans_y, 0);
	glTranslatef(0.5, 0.5, 0);
	glScalef(1./zoom_factor, 1./zoom_factor, 1./zoom_factor);
	glTranslatef(-.5,-.5, 0);

	glGetFloatv(GL_MODELVIEW_MATRIX, inverse_tran);

	glPopMatrix();
}
void GlView::transform_fun()
{
	glTranslatef(trans_x, trans_y, 0);
	glTranslatef(0.5, 0.5, 0);
	glScalef(zoom_factor, zoom_factor, zoom_factor);
	glTranslatef(-.5,-.5, 0);
}
void  GlView::HsvRgb( float hsv[3], float rgb[3] )
{
	float h, s, v;			// hue, sat, value
	float r, g, b;			// red, green, blue
	float i, f, p, q, t;		// interim values

	// guarantee valid input:
	h = hsv[0] / 60.;
	while( h >= 6. )	h -= 6.;
	while( h <  0. ) 	h += 6.;

	s = hsv[1];
	if( s < 0. )
		s = 0.;
	if( s > 1. )
		s = 1.;

	v = hsv[2];
	if( v < 0. )
		v = 0.;
	if( v > 1. )
		v = 1.;

	// if sat==0, then is a gray:
	if( s == 0.0 )
	{
		rgb[0] = rgb[1] = rgb[2] = v;
		return;
	}

	// get an rgb from the hue itself:
	i = floor( h );
	f = h - i;
	p = v * ( 1. - s );
	q = v * ( 1. - s*f );
	t = v * ( 1. - ( s * (1.-f) ) );

	switch( (int) i )
	{
	case 0:
		r = v;	g = t;	b = p;
		break;

	case 1:
		r = q;	g = v;	b = p;
		break;

	case 2:
		r = p;	g = v;	b = t;
		break;

	case 3:
		r = p;	g = q;	b = v;
		break;

	case 4:
		r = t;	g = p;	b = v;
		break;

	case 5:
		r = v;	g = p;	b = q;
		break;
	}

	rgb[0] = r;
	rgb[1] = g;
	rgb[2] = b;
}


int GlView::without_anti_aliasing(GLenum mode)
{
	glClearColor(0.93, 0.93, 0.87, 1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glPushMatrix ();
	transform_fun();
	if(showIBFVOn) /*visualize tensor field*/
	{
		glEnable(GL_TEXTURE_2D);
		glShadeModel(GL_FLAT);
		/*using quad mesh 09/25/2007*/
        render_majorfield_quad();  /*  showing major field only  */

	}
	else
	{
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		glDisable(GL_TEXTURE_2D);
	}					
	glDisable(GL_TEXTURE_2D);
	glEnable(GL_COLOR_MATERIAL);
	glLineWidth(2.0);

	if(showRegularElemOn)
		display_tenRegElem(mode);

	if(showGridOn)
		display_design_grid();

	if(showSingularitiesOn)
		display_degenerate_pts(mode);

	if (showMajorTenLine)
		display_major_tenlines(mode);

    drawDirMap();

	glPopMatrix ();
	glDisable(GL_COLOR_MATERIAL);
  	glEnable(GL_TEXTURE_2D);
	glutSwapBuffers();
	return true;	
}
void GlView::render_majorfield_quad()
{

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity(); 
	gluOrtho2D(0, 1, 0, 1);

	major_vis_quad();

	glPopMatrix();

    render_tensor_final(0);

	glDisable(GL_TEXTURE_2D);
}



void GlView::render_tensor_final(unsigned char majororminor)
{
	/*use the mesh to display the texture instead 09/21/2007*/
	glClearColor(1, 1, 1, 1);
	//glClearColor(0.6, .7, 0.9, 1.);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	if(majororminor == 0)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
			GL_RGB, GL_UNSIGNED_BYTE, major_tex);
	}
	else if(majororminor == 1)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
			GL_RGB, GL_UNSIGNED_BYTE, minor_tex);
	}

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0);  glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0);  glVertex2f(1.0, 1.0);
	glEnd();
}
void render_a_map(unsigned char *map)
{
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);

	if(map==NULL) return;

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGB, GL_UNSIGNED_BYTE, map);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0);  glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0);  glVertex2f(1.0, 1.0);
	glEnd();
}

void GlView::major_vis_quad()
{
	/*reset the view point here 11/09/2007*/
	glViewport(0, 0, (GLsizei)NPIX, (GLsizei)NPIX);

	/*rendering the positive x direction*/
	glBindTexture(GL_TEXTURE_2D, tentextnames[0]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGB, GL_UNSIGNED_BYTE, major_tex1);

	render_ibfv_tens_quad(false, false);

	/*save image*/
	glDisable(GL_BLEND);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex1);

	/***************************************************************/

	/*rendering the positive y direction*/
	major_iframe--;
	glBindTexture(GL_TEXTURE_2D, tentextnames[1]);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, NPIX, NPIX, 0,
		GL_RGB, GL_UNSIGNED_BYTE, major_tex2);

	render_ibfv_tens_quad(false, true);

	glDisable(GL_BLEND);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex2);
	/***************************************************************/

	/*blend them*/

	int i, j;
	for(i=0; i<NPIX; i++) /*y direction*/
		for(j=0; j<NPIX; j++)
		{
			major_temp[i][j][0] = major_tex1[i][j][0];
			major_temp[i][j][1] = major_tex1[i][j][1];
			major_temp[i][j][2] = major_tex1[i][j][2];
			major_temp[i][j][3] = major_alpha_map[i][j][0];
		}

    for(i=0; i<NPIX; i++) /*x direction*/
        for(j=0; j<NPIX; j++)
        {
            minor_temp[i][j][0] = major_tex2[i][j][0];
            minor_temp[i][j][1] = major_tex2[i][j][1];
            minor_temp[i][j][2] = major_tex2[i][j][2];
            minor_temp[i][j][3] = 255-major_alpha_map[i][j][0];
        }
    glEnable(GL_BLEND);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, NPIX, NPIX, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, major_temp);
    render_tensor_blend();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, NPIX, NPIX, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, minor_temp);
    render_tensor_blend();

    ////

    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_tex);

    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGBA, GL_UNSIGNED_BYTE, major_temp);
    glReadPixels(0, 0, NPIX, NPIX, GL_RGBA, GL_UNSIGNED_BYTE, minor_temp);

    glViewport(0, 0, (GLsizei)REALWINSIZE, (GLsizei)REALWINSIZE);
			//glViewport(0, 0, (GLsizei)(REALWINSIZE*zoom_factor), (GLsizei)(REALWINSIZE*zoom_factor));
}
void GlView::render_ibfv_tens_quad(bool major_minor, bool x_y)
{
	int i, j;
	QuadCell *face;
	QuadVertex *vert;
	double px, py;

	//glViewport(0, 0, (GLsizei) NPIX, (GLsizei) NPIX);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity(); 
	gluOrtho2D(0, 1, 0, 1);


	/*major*/
	if(!major_minor) /*major field*/
	{
		if(!x_y) /*positive x*/
		{
			for (i=0; i<quadmesh->nfaces; i++) {
				face = quadmesh->quadcells[i];
                glBegin(GL_POLYGON);
				for (j=0; j<face->nverts; j++) {
					vert = quadmesh->quad_verts[face->verts[j]];
					glTexCoord2f(vert->x, vert->y);

					if(vert->major_cos)
					{
						px = vert->x - vert->major.entry[0];
						py = vert->y - vert->major.entry[1];
					}
					else
					{
						px = vert->x + vert->major.entry[0];
						py = vert->y + vert->major.entry[1];
					}

					glVertex2f(px, py);
				}
				glEnd();
			}
		}
		else  /*positive y*/
		{
			for (i=0; i<quadmesh->nfaces; i++) {
				face = quadmesh->quadcells[i];
                glBegin(GL_POLYGON);
				for (j=0; j<face->nverts; j++) {
					vert = quadmesh->quad_verts[face->verts[j]];
					glTexCoord2f(vert->x, vert->y);
					if(vert->major_sin)
					{
						px = vert->x - vert->major.entry[0];
						py = vert->y - vert->major.entry[1];
					}
					else
					{
						px = vert->x + vert->major.entry[0];
						py = vert->y + vert->major.entry[1];
					}
					glVertex2f(px, py);
				}
				glEnd();
			}
		}
	}
	else  /*minor field*/
	{
		if(!x_y) /*positive x*/
		{
			for (i=0; i<quadmesh->nfaces; i++) {
				face = quadmesh->quadcells[i];
				glBegin(GL_POLYGON);
				for (j=0; j<face->nverts; j++) {
					vert = quadmesh->quad_verts[face->verts[j]];
					glTexCoord2f(vert->x, vert->y);
					if(vert->minor_cos)
					{
						px = vert->x - vert->minor.entry[0];
						py = vert->y - vert->minor.entry[1];
					}
					else
					{
						px = vert->x + vert->minor.entry[0];
						py = vert->y + vert->minor.entry[1];
					}
					glVertex2f(px, py);
				}
				glEnd();
			}
		}
		else  /*positive y*/
		{
			for (i=0; i<quadmesh->nfaces; i++) {
				face = quadmesh->quadcells[i];
				glBegin(GL_POLYGON);
				for (j=0; j<face->nverts; j++) {
					vert = quadmesh->quad_verts[face->verts[j]];
					glTexCoord2f(vert->x, vert->y);
					if(vert->minor_sin)
					{
						px = vert->x - vert->minor.entry[0];
						py = vert->y - vert->minor.entry[1];
					}
					else
					{
						px = vert->x + vert->minor.entry[0];
						py = vert->y + vert->minor.entry[1];
					}
					glVertex2f(px, py);
				}
				glEnd();
			}
		}
	}

	if(!major_minor)
		major_iframe ++;
	else
		minor_iframe ++;

	glEnable(GL_BLEND); 

	/*double check the following codes*/
    glCallList(major_iframe % Npat + 1+200);

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0,  0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0,  ten_tmax); glVertex2f(0.0, 1.0);
	glTexCoord2f(ten_tmax, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(ten_tmax, ten_tmax); glVertex2f(1.0, 1.0);
	glEnd();

	glPopMatrix();

}
void GlView::render_tensor_blend()
{
	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0);  glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0);  glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0);  glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0);  glVertex2f(1.0, 1.0);
	glEnd();
}
void GlView::drawSolidRect_size(double cx, double cy, double R)
{
	glBegin(GL_POLYGON);
	glVertex2f(cx-R, cy-R);
	glVertex2f(cx-R, cy+R);
	glVertex2f(cx+R, cy+R);
	glVertex2f(cx+R, cy-R);
	glEnd();
}
void GlView::drawSolidCircle_size(double cx, double cy, double R)
{
	int i;
	double theta, deta ;
	deta = 2 * M_PI/49.;
	double x, y;
	theta = 0.;
	glBegin(GL_POLYGON);
	for(i = 0; i < 50; i++, theta += deta)
	{
		x =  cx + R * cos(theta);
		y =  cy + R * sin(theta);

		glVertex2f(x, y);
	}
	glEnd();
}

////display the regular elements using arrows
void GlView::display_tenRegElem(GLenum mode)
{
	glLineWidth(2.);
	for(int i = 0; i < nten_regelems; i++)
	{
		if(ten_regularelems[i].ID>0)
		{
			////Display the arrow
			if(mode == GL_SELECT)
				glLoadName(ten_regularelems[i].ID);

			if(ten_regularelems[i].deleted)
				continue;

			////perform the user defined transformation for editing

			if(ten_regularelems[i].type == 0) ////basic regular element
				glColor3f(0, 1, 1);
			else if(ten_regularelems[i].type == 1) ////convergent element
				glColor3f(1, 0.5, 0);
			//glColor3f(1, 1, 0);       //for the paper
			else                         ////divergent element
				glColor3f(0, 1, 0.5);

			/*we draw two line segments*/
			glBegin(GL_LINES);
			glVertex2f(ten_regularelems[i].base[0], ten_regularelems[i].base[1]);
			glVertex2f(ten_regularelems[i].end[0], ten_regularelems[i].end[1]);
			glEnd();

			glBegin(GL_LINES);
			glVertex2f(ten_regularelems[i].base[0], ten_regularelems[i].base[1]);
			glVertex2f(ten_regularelems[i].base[0]-ten_regularelems[i].Direct.entry[0], 
				ten_regularelems[i].base[1]-ten_regularelems[i].Direct.entry[1]);
			glEnd();

			/*draw three control points*/

			//if(mode == GL_SELECT)
			// glLoadName(NAMEOFREGCONTROL+1);   ////control point at the base
			drawSolidRect_size(ten_regularelems[i].base[0], ten_regularelems[i].base[1], 0.006/zoom_factor);

			//if(mode == GL_SELECT)
			// glLoadName(NAMEOFREGCONTROL+2);   ////control point at the base
			drawSolidCircle_size(ten_regularelems[i].base[0]-ten_regularelems[i].Direct.entry[0], 
				ten_regularelems[i].base[1]-ten_regularelems[i].Direct.entry[1], 0.006/zoom_factor);

			//if(mode == GL_SELECT)
			// glLoadName(NAMEOFREGCONTROL+3);   ////control point at the end
			drawSolidCircle_size(ten_regularelems[i].end[0], ten_regularelems[i].end[1], 0.006/zoom_factor);

		}
	}
}
void GlView::display_design_grid()
{
	glColor3f(0.8, 0.8, 0.5);
	glLineWidth(1.);

	QuadCell* face;
	QuadVertex *v;
	int j;
	for(int i=0;i<quadmesh->nfaces;i++)
	{
		face=quadmesh->quadcells[i];

		glBegin(GL_LINE_LOOP);
		for(j=0;j<face->nverts;j++)
		{
			v=quadmesh->quad_verts[face->verts[j]];
			glVertex2f(v->x, v->y);
		}
		glEnd();
	}
}
void GlView::display_degenerate_pts(GLenum mode)
{
	int i, singular_id = 0;

	for(i = 0; i < ndegpts; i++)
	{
		if(degpts[i].type == 0) /*wedge*/
			glColor3f(1, 0, 0);
		else if(degpts[i].type == 1) /*trisector*/
			glColor3f(0, 1, 0);
		else if(degpts[i].type == 2) /*node*/
			glColor3f(1, 1, 0);
		else if(degpts[i].type == 3) /*center*/
			glColor3f(1, 0, 1);
		else if(degpts[i].type == 4) /*saddle*/
			glColor3f(0, 0, 1);
		else
			glColor3f(1, 1, 1);

		drawSolidCircle_size(degpts[i].gcx, degpts[i].gcy, 0.006/zoom_factor);

		glColor3f(0, 0, 0);
		glLineWidth(1.4);
		draw_hollow_circle_size(degpts[i].gcx, degpts[i].gcy, 0.0065/zoom_factor);
	}
}

void GlView::draw_hollow_circle_size(double cx, double cy, double R)
{
	int i;
	double theta, deta ;
	deta = 2 * M_PI/49.;
	double x, y;
	theta = 0.;
	glBegin(GL_LINE_LOOP);
	for(i = 0; i < 50; i++, theta += deta)
	{
		x =  cx + R * cos(theta);
		y =  cy + R * sin(theta);

		glVertex2f(x, y);
	}
	glEnd();
}
int GlView::DrawGLScene(GLenum mode)					// Here's Where We Do All The Drawing
{
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
	glDisable(GL_LIGHTING);
	without_anti_aliasing(mode);
	return 1;
}
void GlView::makePatterns(void) 
{ 
	int lut[256];
	int phase[NPN][NPN];
	GLubyte pat[NPN][NPN][4];
	int i, j, k, t;

	for (i = 0; i < 256; i++) lut[i] = i < 127 ? 0 : 255;
	for (i = 0; i < NPN; i++)
		for (j = 0; j < NPN; j++) phase[i][j] = rand() % 256; 

	for (k = 0; k < Npat; k++) {
		t = k*256/Npat;
		for (i = 0; i < NPN; i++) 
			for (j = 0; j < NPN; j++) {
				pat[i][j][0] =
					pat[i][j][1] =
					pat[i][j][2] = lut[(t + phase[i][j]) % 255];
				pat[i][j][3] = alpha;
			}
			glNewList(k + 1, GL_COMPILE);
			glTexImage2D(GL_TEXTURE_2D, 0, 4, NPN, NPN, 0, 
				GL_RGBA, GL_UNSIGNED_BYTE, pat);
			glEndList();
	}
}
void GlView::save_cur_field()
{
	int i;
	for(i=0;i<quadmesh->nverts;i++)
	{
		pre_tenfield[i].set(quadmesh->quad_verts[i]->Jacobian);
	}
}
//////////////////////////////////////////////////////////////////////////
void GlView::OnLButtonDown(int x, int y){

	double s, t;

	GLdouble position[3];
	m_LeftButtonDown_posX=x;
	m_LeftButtonDown_posY=y;
	ScreenToWorld(position, x,y);
	s = position[0];
	t = position[1];
	if( ((s < 0) || (s > 1) || (t < 0) || (t > 1)))
	{
		return;
	}
	s=(s-.5-trans_x)/zoom_factor+.5;
	t=(t-.5-trans_y)/zoom_factor+.5;
	set_ten_regBasis(s, t, 0);  /*set the basis of the regular element*/
	m_LeftButtonDown = true;
}
void GlView::ScreenToWorld(double position[3], int x,int y)
{
	int viewport[4];
	GLdouble modelMatrix[16];
	GLdouble projMatrix[16];

	glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	glGetIntegerv(GL_VIEWPORT,viewport);

	gluUnProject(
		x,
		y,
		0,
		modelMatrix,
		projMatrix,
		viewport,
		//the next 3 parameters are the pointers to the final object
		//coordinates. Notice that these MUST be double's
		&position[0], //-> pointer to your own position (optional)
		&position[1], // id
		&position[2] // id
	);

	//position[0] = position[0]-0.02;
	//position[1] = 1 - position[1] + 0.02;
	position[0] = position[0]-0.01;
	position[1] = 1 - position[1]+0.01;
}
void GlView::set_ten_regBasis(double x, double y, int type)
{
	////Add to the regular elements list
	if(nten_regelems >= curMaxNumTenRegElems-1 )
	{
		//Allocate new space for the element list
		curMaxNumTenRegElems += 50;
		ten_regularelems = (TenRegularElem*)realloc(ten_regularelems, sizeof(TenRegularElem) * curMaxNumTenRegElems);
		if(ten_regularelems == NULL)
			exit(-1);

	}
	ten_regularelems[nten_regelems].end[0] = ten_regularelems[nten_regelems].base[0] = x;
	ten_regularelems[nten_regelems].end[1] = ten_regularelems[nten_regelems].base[1] = y;
	ten_regularelems[nten_regelems].ID = NAMEOFREGELEM + nten_regelems+1;
	ten_regularelems[nten_regelems].type = type;
	ten_regularelems[nten_regelems].Direct.set(0, 0);

	////Initialize the transformation parameters for this regular element
	ten_regularelems[nten_regelems].transform_matrix.setIdentity();
	//ten_regularelems[nten_regelems].transposeRot.setIdentity();

	ten_regularelems[nten_regelems].rotang = 0;
	ten_regularelems[nten_regelems].s = 1;

	ten_regularelems[nten_regelems].deleted=false;

	/*mark which region the design element belongs to 11/21/2007*/
	ten_regularelems[nten_regelems].which_region=get_region_id(x, y);

	/*  may be a bug 1/18/2008 */
	cur_chosen_region = ten_regularelems[nten_regelems].which_region;

	nten_regelems ++;
}
void GlView::setRegularElem(double base_x, double base_y,double end_x,double end_y){
	set_ten_regBasis(base_x,base_y,0);
    set_ten_regDir(end_x,end_y);

    cal_tensorvals_quad_inReg();
    cal_all_eigenvecs_quad();

    init_degpts();

    /*calculate the alpha map here*/
    render_alpha_map_quad(false);
    render_alpha_map_quad(true);

    locate_degpts_cells_tranvec_quad();

}

unsigned char GlView::get_region_id(double x, double y)
{
	int i=(x-quadmesh->xstart)/quadmesh->xinterval;
	int j=(y-quadmesh->ystart)/quadmesh->yinterval;

	if(i>=quadmesh->XDIM-1) i=quadmesh->XDIM-2;
	if(j>=quadmesh->YDIM-1) j=quadmesh->YDIM-2;

	int cellid=j*(quadmesh->XDIM-1)+i;
	return quadmesh->quadcells[cellid]->which_region;
}
void GlView::OnLButtonUp(int x, int y)
{
	// TODO: Add your message handler code here and/or call default
	double  s, t, s_down, t_down;
	GLdouble position[3];

	ScreenToWorld(position, x,y);
	s_down = position[0];
	t_down = position[1];

	ScreenToWorld(position, m_LeftButtonDown_posX,m_LeftButtonDown_posY);
	s = position[0];
	t = position[1];

	if ((s == s_down) && (t == t_down))
	{
		m_LeftButtonDown = false;
		return;
	}


	if( (s < 0) || (s > 1) || (t < 0) || (t > 1))
	{
		m_LeftButtonDown = false;
		return;
	}
	m_LeftButtonDown = false;
}
void GlView::OnMouseMove(int x,int y)
{
	// TODO: Add your message handler code here and/or call default
	double s, t;

	GLdouble position[3];

	ScreenToWorld(position, x,y);
	s = position[0];
	t = position[1];

	//if(pan_s_old==s && t== pan_t_old && m_MiddleButtonDown == true)
	//	return;
	
	if( (s < 0) || (s > 1) || (t < 0) || (t > 1))
	{
		if( s < 0) s = 0;
		if( s > 1) s = 1;
		if( t < 0) t = 0;
		if( t > 1) t = 1;
	}
	
	double pan_dx=s-pan_s_old;
	double pan_dy=t-pan_t_old;
	pan_s_old=s;
	pan_t_old=t;


	s=(s-.5-trans_x)/zoom_factor+.5;
	t=(t-.5-trans_y)/zoom_factor+.5;

	//if ((s == s_old) && (t == t_old) && m_LeftButtonDown == TRUE)
	//	return;
	if ((fabs(s - s_old)<1.e-7 && fabs(t - t_old)<1.e-7) && m_LeftButtonDown == true)
		return;
	
	double dx = s - s_old, dy = t - t_old;

			//dx=(dx-.5-trans_x)/zoom_factor+.5;
			//dy=(dy-.5-trans_y)/zoom_factor+.5;
	
	save_cur_field();

	if ( m_LeftButtonDown == true) /*it is tensor regular element design*/
	{
		/*record the direction and the end point for the direction and the strength of the element*/
		set_ten_regDir(s, t);  /*set the end point and direction of the regular element*/
	
		cal_tensorvals_quad_inReg(/*cur_chosen_region*/);
		cal_all_eigenvecs_quad();

		init_degpts();

		/*calculate the alpha map here*/
		render_alpha_map_quad(false);
		render_alpha_map_quad(true);
			
		locate_degpts_cells_tranvec_quad();
        reset_major_path();
        listen_to_robot_loc();
       // test();
        cancelPath();
        addRobotWayPoint();
	}

	s_old = s;
	t_old = t;
	DrawGLScene(GL_RENDER);
}
void GlView::set_ten_regDir(double x, double y)
{
	////Update the Cur_regularID here

	ten_regularelems[nten_regelems - 1].end[0] = x ;
	ten_regularelems[nten_regelems - 1].end[1] = y ;

	ten_regularelems[nten_regelems - 1].Direct.entry[0] = x - ten_regularelems[nten_regelems - 1].base[0];
	ten_regularelems[nten_regelems - 1].Direct.entry[1] = y - ten_regularelems[nten_regelems - 1].base[1];

	ten_regularelems[nten_regelems - 1].rotang = 
		atan2(ten_regularelems[nten_regelems - 1].Direct.entry[1],
		ten_regularelems[nten_regelems - 1].Direct.entry[0]);


}
void GlView::cal_tensorvals_quad_inReg(/*int regionid*/)
{
	int i;
	QuadVertex *v;
	double t[4]={0.};

	for(i=0; i<quadmesh->nverts; i++)
	{
		v = quadmesh->quad_verts[i];

		if(!v->inland)
			continue;

		get_tensor_inReg(v->x, v->y, t, v->which_region, v->inbrushregion);
		if(please_comb_prefield/*sharedvars.SelStreetRegToEditOn*/)
		{
			v->Jacobian.entry[0][0] = 0.1*pre_tenfield[i].entry[0][0]+t[0];
			v->Jacobian.entry[0][1] = 0.1*pre_tenfield[i].entry[0][1]+t[1];
			v->Jacobian.entry[1][0] = 0.1*pre_tenfield[i].entry[1][0]+t[2];
			v->Jacobian.entry[1][1] = 0.1*pre_tenfield[i].entry[1][1]+t[3];
		}
		else{
			//get_tensor_inReg(v->x, v->y, t, v->which_region, v->inbrushregion);
			v->Jacobian.entry[0][0] = t[0];
			v->Jacobian.entry[0][1] = t[1];
			v->Jacobian.entry[1][0] = t[2];
			v->Jacobian.entry[1][1] = t[3];
		}
	}
}


void GlView::get_tensor_inReg(double x, double y, double t[4], int regionid, bool inbrush)
{
	int i;
	double  dx, dy, vx, vy, t00, t01, t10, t11, r=0.;
	double d;

	icMatrix3x3 tempJacobian, transposerot;
	double ang;

	vx = vy = 0.;

	t[0]=t[1]=t[2]=t[3]=0.;

	///Combine all the degenerate elements 
	for(i = 0; i < ntenelems; i++)
	{
		if(ten_designelems[i].ID >= 0 && !ten_designelems[i].deleted
			&& ten_designelems[i].which_region==regionid)
		{
			dx = x - ten_designelems[i].centerx;
			dy = y - ten_designelems[i].centery;

			r  = dx*dx + dy*dy; 
			d = exp(-1000*r);

			if (r < DistanceThreshold)   r = DistanceThreshold;

			tempJacobian.set(ten_designelems[i].transform_matrix);

			ang = ten_designelems[i].rotang;

			transposerot.set(cos(ang), sin(ang), 0,
				-sin(ang),  cos(ang), 0,
				0,0,1);

			tempJacobian.rightMultiply(transposerot);

			if(ten_designelems[i].type == 0) /*wedge*/
			{
				t00 = (dx * tempJacobian.entry[0][0] + dy * tempJacobian.entry[0][1])/r;  
				t01 = (dy * tempJacobian.entry[0][0] - dx * tempJacobian.entry[0][1])/r;
				t10 = (dx * tempJacobian.entry[1][0] + dy * tempJacobian.entry[1][1])/r;
				t11 = (dy * tempJacobian.entry[1][0] - dx * tempJacobian.entry[1][1])/r;
			}
			else if(ten_designelems[i].type == 1) /*trisector*/
			{
				t00 = (dx * tempJacobian.entry[0][0] - dy * tempJacobian.entry[0][1])/r;  
				t01 = (-dy * tempJacobian.entry[0][0] - dx * tempJacobian.entry[0][1])/r;
				t10 = (dx * tempJacobian.entry[1][0] - dy * tempJacobian.entry[1][1])/r;
				t11 = (-dy * tempJacobian.entry[1][0] - dx * tempJacobian.entry[1][1])/r;
			}
			else if(ten_designelems[i].type == 2) /*node*/
			{
				t00 = ((dx*dx-dy*dy)*tempJacobian.entry[0][0]+2*dx*dy*tempJacobian.entry[0][1])/(r*sqrt(r));
				t01 = (2*dx*dy*tempJacobian.entry[0][0]-(dx*dx-dy*dy)*tempJacobian.entry[0][1])/(r*sqrt(r));
				t10 = ((dx*dx-dy*dy)*tempJacobian.entry[1][0]+2*dx*dy*tempJacobian.entry[1][1])/(r*sqrt(r));
				t11 = (2*dx*dy*tempJacobian.entry[1][0]-(dx*dx-dy*dy)*tempJacobian.entry[1][1])/(r*sqrt(r));

			}
			else if(ten_designelems[i].type == 3) /*center*/
			{
				t00 = ((dy*dy-dx*dx)*tempJacobian.entry[0][0]-2*dx*dy*tempJacobian.entry[0][1])/(r*sqrt(r));
				t01 = (-2*dx*dy*tempJacobian.entry[0][0]+(dx*dx-dy*dy)*tempJacobian.entry[0][1])/(r*sqrt(r));
				t10 = ((dy*dy-dx*dx)*tempJacobian.entry[1][0]-2*dx*dy*tempJacobian.entry[1][1])/(r*sqrt(r));
				t11 = (-2*dx*dy*tempJacobian.entry[1][0]+(dx*dx-dy*dy)*tempJacobian.entry[1][1])/(r*sqrt(r));
			}
			else if(ten_designelems[i].type == 4) /*saddle*/
			{
				t00 = ((dx*dx-dy*dy)*tempJacobian.entry[0][0]-2*dx*dy*tempJacobian.entry[0][1])/(r*sqrt(r));
				t01 = (-2*dx*dy*tempJacobian.entry[0][0]-(dx*dx-dy*dy)*tempJacobian.entry[0][1])/(r*sqrt(r));
				t10 = ((dx*dx-dy*dy)*tempJacobian.entry[1][0]-2*dx*dy*tempJacobian.entry[1][1])/(r*sqrt(r));
				t11 = (-2*dx*dy*tempJacobian.entry[1][0]-(dx*dx-dy*dy)*tempJacobian.entry[1][1])/(r*sqrt(r));
			}

			/*you may also need to multiply the transpose of the transformation matrix*/


			t[0] += t00/sqrt(r)*LOWER;
			t[1] += t01/sqrt(r)*LOWER;
			t[2] += t10/sqrt(r)*LOWER;
			t[3] += t11/sqrt(r)*LOWER;

		}

	}

	/*the following we combine the regular element*/
	icMatrix3x3 regten, temp;
   // ROS_INFO("number of regelems is: %d",nten_regelems);
	for(i = 0; i < nten_regelems; i++)
	{
		if(ten_regularelems[i].ID > 0 && !ten_regularelems[i].deleted
			&& ten_regularelems[i].which_region == regionid)
		{
			dx = x - ten_regularelems[i].base[0];
			dy = y - ten_regularelems[i].base[1];

			r  = dx*dx + dy*dy; 

			if (r < DistanceThreshold)   r = DistanceThreshold;

			if(ten_regularelems[i].type == 0) ////regular element
			{
				/*the creation of regular element before 09/26/2007*/
				double strength = length(ten_regularelems[i].Direct);

				t[0] += strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[1] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[2] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[3] += -strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));

			}

		}

		else if(ten_regularelems[i].ID > 0 && 
			!ten_regularelems[i].deleted && ten_regularelems[i].which_region == 0
			&& inbrush)
		{
			dx = x - ten_regularelems[i].base[0];
			dy = y - ten_regularelems[i].base[1];

			r  = dx*dx + dy*dy; 

			if (r < DistanceThreshold)   r = DistanceThreshold;

			if(ten_regularelems[i].type == 0) ////regular element
			{
				/*the creation of regular element before 09/26/2007*/
				double strength = length(ten_regularelems[i].Direct);

				t[0] += strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[1] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[2] += strength*sin(2*ten_regularelems[i].rotang)/(r*sqrt(r));
				t[3] += -strength*cos(2*ten_regularelems[i].rotang)/(r*sqrt(r));

			}
		}
	}

}

void GlView::init_ten_designelems()
{
	/*initialize singular design element*/
	if(ten_designelems == NULL)
	{
		ten_designelems=(Degenerate_Design *)malloc(sizeof(Degenerate_Design)*curMaxNumTenDesignElems);
		if(ten_designelems == NULL)
			exit(-1);
	}
	ntenelems = 0;

	init_regular_ten_designelems();
}
void GlView::cal_all_eigenvecs_quad()
{
	int i;

	for(i=0; i<quadmesh->nverts; i++)
	{
		if(quadmesh->quad_verts[i]->inland)
			cal_eigenvecs_onevert_quad(i);
	}

	/*normalize the major and minor field*/
	normalized_tensorfield_quad();
}

void GlView::cal_eigenvecs_onevert_quad(int ver)
{
	QuadVertex *v=quadmesh->quad_verts[ver];
	double evalues[2];
	icVector2 ev[2];

	if(fabs(v->Jacobian.entry[0][0])<=1.e-7
		&&fabs(v->Jacobian.entry[0][1])<=1.e-7
		&&fabs(v->Jacobian.entry[1][0])<=1.e-7
		&&fabs(v->Jacobian.entry[1][1])<=1.e-7)
	{
		v->major.set(0,0);
		v->minor.set(0,0);
		v->major_ang=0;
		v->minor_ang=0;
		return;
	}

	cal_eigen_vector_sym(v->Jacobian, ev);

	v->major = ev[0];
	v->minor = ev[1];

	/*compute the angle*/
	v->major_ang = atan2(v->major.entry[1], v->major.entry[0]);
	v->minor_ang = atan2(v->minor.entry[1], v->minor.entry[0]);

	/*save the angle of major field for degenerate points/singularity detection*/
	//if(v->major_ang<0)
	//	v->tensor_major_ang = M_PI+v->major_ang;
	//else
		v->tensor_major_ang = v->major_ang;
		
	/*obtain the multiplied vector, we will use this obtained
	vector field to extract singularities*/
	v->tran_vec.set(cos(2*v->tensor_major_ang), sin(2*v->tensor_major_ang));


	/*transfer to cos^2*/
	double major_ang_cos = cos(v->major_ang);
	double major_ang_sin = sin(v->major_ang);

	v->major_ang = major_ang_cos*major_ang_cos;
	if(major_ang_cos<0)
		v->major_cos = true;
	else
		v->major_cos = false;
	if(major_ang_sin<0)
		v->major_sin = true;
	else
		v->major_sin = false;

	double minor_ang_cos = cos(v->minor_ang);
	double minor_ang_sin = sin(v->minor_ang);

	v->minor_ang = minor_ang_cos*minor_ang_cos;
	//v->minor_ang = minor_ang_sin*minor_ang_sin;
	if(minor_ang_cos<0)
		v->minor_cos = true;
	else
		v->minor_cos = false;
	if(minor_ang_sin<0)
		v->minor_sin = true;
	else
		v->minor_sin = false;

}
void GlView::cal_eigen_vector_sym(icMatrix2x2 m, icVector2 ev[2])
{
	/*first get the deviator of m*/
	icMatrix2x2 dev;
	double half_trace = 0.5*(m.entry[0][0]+m.entry[1][1]);
	dev.entry[0][0] = m.entry[0][0]-half_trace;
	dev.entry[1][1] = m.entry[1][1]-half_trace;
	dev.entry[0][1] = m.entry[0][1];
	dev.entry[1][0] = m.entry[1][0];

	/*compute the eigen vectors*/
	double theta = atan2(dev.entry[0][1], dev.entry[0][0]);

	//if(theta < 0) theta += 2*M_PI;

	/*major eigen vector*/
	ev[0].entry[0] = cos(theta/2.);
	ev[0].entry[1] = sin(theta/2.);

	//ev[0] = half_trace*ev[0];

	/*minor eigen vector*/
	ev[1].entry[0] = cos((theta+M_PI)/2.);
	ev[1].entry[1] = sin((theta+M_PI)/2.);

	//ev[1] = half_trace*ev[1];
}
void GlView::normalized_tensorfield()
{
	/*normalize the major and minor field*/
	int i;
	double r;
	Vertex *cur_v;

	for(i = 0; i < Object.nverts; i++)
	{
		cur_v = Object.vlist[i];

		/*normalize major field*/
		r = length(cur_v->major);
		r *= r;
		if (r < DistanceThreshold) 
		{
			r = DistanceThreshold;
			cur_v->major *= ten_dmax/r; 
		}
		r = length(cur_v->major);
		r *= r;
		if (r > ten_dmax*ten_dmax) { 
			r  = sqrt(r); 
			cur_v->major *= ten_dmax/r; 
		}

		/*normalize minor field*/
		r = length(cur_v->minor);
		r *= r;
		if (r < DistanceThreshold) 
		{
			r = DistanceThreshold;
			cur_v->minor *= ten_dmax/r; 
		}
		r = length(cur_v->minor);
		r *= r;
		if (r > ten_dmax*ten_dmax) { 
			r  = sqrt(r); 
			cur_v->minor *= ten_dmax/r; 
		}
	}
}
void GlView::normalized_tensorfield_quad()
{
	/*normalize the major and minor field*/
	int i;
	double r;
	QuadVertex *cur_v;

	for(i = 0; i < quadmesh->nverts; i++)
	{
		cur_v = quadmesh->quad_verts[i];

		/*normalize major field*/
		r = length(cur_v->major);
		r *= r;
		if (r < DistanceThreshold) 
		{
			r = DistanceThreshold;
			cur_v->major *= ten_dmax/r; 
		}
		r = length(cur_v->major);
		r *= r;
		if (r > ten_dmax*ten_dmax) { 
			r  = sqrt(r); 
			cur_v->major *= ten_dmax/r; 
		}

		/*normalize minor field*/
		r = length(cur_v->minor);
		r *= r;
		if (r < DistanceThreshold) 
		{
			r = DistanceThreshold;
			cur_v->minor *= ten_dmax/r; 
		}
		r = length(cur_v->minor);
		r *= r;
		if (r > ten_dmax*ten_dmax) { 
			r  = sqrt(r); 
			cur_v->minor *= ten_dmax/r; 
		}
	}
}
void GlView::render_alpha_map_quad(bool major_minor)
{
	/**/
	int i, j;
	QuadCell *face;
	QuadVertex *vert;

	glViewport(0, 0, (GLsizei)NPIX, (GLsizei)NPIX);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glShadeModel(GL_SMOOTH);
	if(!major_minor)
	{
		for (i=0; i<quadmesh->nfaces; i++) {
			face = quadmesh->quadcells[i];
			glBegin(GL_POLYGON);
			for (j=0; j<face->nverts; j++) {
				vert = quadmesh->quad_verts[face->verts[j]];
				glColor3f(vert->major_ang,
					vert->major_ang,
					vert->major_ang);

				glVertex2f(vert->x, vert->y);
			}
			glEnd();
		}

		/*copy to the major_alpha_map*/
		glReadBuffer(GL_BACK);
		glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, major_alpha_map);
	}
	else
	{
		for (i=0; i<quadmesh->nfaces; i++) {
			face = quadmesh->quadcells[i];
			glBegin(GL_POLYGON);
			for (j=0; j<face->nverts; j++) {
				vert = quadmesh->quad_verts[face->verts[j]];
				glColor3f(vert->minor_ang,
					vert->minor_ang,
					vert->minor_ang);
				glVertex2f(vert->x, vert->y);
			}
			glEnd();
		}

		/*copy to the minor_alpha_map*/
		glReadBuffer(GL_BACK);
		glReadPixels(0, 0, NPIX, NPIX, GL_RGB, GL_UNSIGNED_BYTE, minor_alpha_map);
	}
	glDisable(GL_BLEND);
	glViewport(0, 0, (GLsizei)REALWINSIZE, (GLsizei)REALWINSIZE);
	//glViewport(0, 0, (GLsizei)(REALWINSIZE*zoom_factor), (GLsizei)(REALWINSIZE*zoom_factor));
}
/*we use quad mesh to locate the degenerate points*/
void GlView::locate_degpts_cells_tranvec_quad(void)
{
	unsigned int i, j;
	QuadCell *face;
	QuadVertex *v;
	icVector2 vec[4];      //vectors at four vertices
	double  theta[4];      //for storing the angles between two vector for Gauss circle judgement

	double  vec_ang[4];  //the angle for each vector under the polar frame of current triangle
	double  ang_sum;

	////Initialize
	ndegenerate_tris = 0;

	if(degenerate_tris == NULL)
		degenerate_tris = (int*) malloc(sizeof(int) * MaxNumDegeneratePts); //default range is 200


	////Calculate the Poincare index
	for (i = 0; i < quadmesh->nfaces; i++) {
		face = quadmesh->quadcells[i];

		ang_sum = 0;

		for(j=0; j<face->nverts; j++)
		{
			v = quadmesh->quad_verts[face->verts[j]];
			vec_ang[j] = atan2(v->tran_vec.entry[1], v->tran_vec.entry[0]);
			if(vec_ang[j] < 0) vec_ang[j] += 2 * M_PI;
		}

		for(j = 0; j < face->nverts; j++)
		{
			theta[j] = vec_ang[(j+1)%face->nverts] - vec_ang[j];

			if( theta[j] < -M_PI)
				theta[j] += 2 * M_PI;

			if( theta[j] > M_PI)
				theta[j] -= 2 * M_PI;

			ang_sum += theta[j];
		}

		double index = ang_sum/(2*M_PI);


		/*here we need to allow some numerical errors 09/26/2007*/
		if(fabs(index) >= 1.- 1.e-1)
		{
			//The triangle must have singularities inside, mark it as yellow color
			//Still need to judge whether it is one of current singularities or not
			degenerate_tris[ndegenerate_tris] = i;
			ndegenerate_tris ++;

			if(fabs(index-1)<1e-2)  /*it is a wedge*/
				quadmesh->quadcells[i]->degenerate_type = 0;
			else if(fabs(index+1)<1e-2)  /*it is a trisector*/
				quadmesh->quadcells[i]->degenerate_type = 1;
			else if(fabs(index-2)<1e-2)    /*it is a node/center*/
				quadmesh->quadcells[i]->degenerate_type = 2;
			else if(fabs(index+2)<1e-2)    /*it is a saddle*/
				quadmesh->quadcells[i]->degenerate_type = 3;

			if(ndegenerate_tris >= MaxNumDegeneratePts - 1)
			{
				MaxNumDegeneratePts += 50;
				degenerate_tris = (int*) realloc(degenerate_tris, sizeof(int) * MaxNumDegeneratePts);
				degpts = (DegeneratePt*)realloc(degpts, sizeof(DegeneratePt)*MaxNumDegeneratePts);
			}
		}
	}

	if(ndegenerate_tris>0) /*we find some degenerate triangles*/
		compute_degpts_pos_tranvec_quad();
}

void GlView::compute_degpts_pos_tranvec_quad()
{
	int i;
	double x_cp, y_cp;
	ndegpts = 0;

	for(i=0; i<ndegenerate_tris; i++)
	{
		compute_onedegpt_pos_tranvec_quad(degenerate_tris[i], x_cp, y_cp);

		/*save the information to the degenerate point list*/
		degpts[ndegpts].gcx = x_cp;
		degpts[ndegpts].gcy = y_cp;
		degpts[ndegpts].degpt_index = ndegpts;
		degpts[ndegpts].type = quadmesh->quadcells[degenerate_tris[i]]->degenerate_type;
		degpts[ndegpts].Triangle_ID = degenerate_tris[i];
		quadmesh->quadcells[degenerate_tris[i]]->degpt_index = ndegpts;

		/*compute the separatrices*/
		degpts[ndegpts].nseps = 0;

		ndegpts++;
	}
}

void GlView::compute_onedegpt_pos_tranvec_quad(int cellid, double &x, double &y)
{
	/*first, we find an a along x direction, such that with this coefficient
	the interpolated vectors between v0v1 and v2v3 have opposite direction*/

	/*second, on this a, we find an b along y direction, such that the
	magnitude of v0v1 equal the magnitude of v2v3*/
	QuadCell *qc = quadmesh->quadcells[cellid];

	//QuadVertex *v00 = quadmesh->quad_verts[qc->verts[0]];
	//QuadVertex *v01 = quadmesh->quad_verts[qc->verts[1]];
	//QuadVertex *v10 = quadmesh->quad_verts[qc->verts[2]];
	//QuadVertex *v11 = quadmesh->quad_verts[qc->verts[3]];
	QuadVertex *v00 = quadmesh->quad_verts[qc->verts[0]];
	QuadVertex *v01 = quadmesh->quad_verts[qc->verts[3]];
	QuadVertex *v10 = quadmesh->quad_verts[qc->verts[1]];
	QuadVertex *v11 = quadmesh->quad_verts[qc->verts[2]];

	icVector2 v0v1, v2v3;
	double a, b;

	/*get a: the most difficult step*/
	compute_a_alongx_degptlocate(a, v00->tran_vec, v10->tran_vec, v11->tran_vec, v01->tran_vec,
		v0v1, v2v3);

	/*get b. after first step, v0v1 and v2v3 are opposite to each other*/
	if(fabs(v0v1.entry[0])>1e-8) /*use x direction*/
	{
		b = (v0v1.entry[0])/(v0v1.entry[0]-v2v3.entry[0]);
	}
	else /*use y direction*/
	{
		b = (v0v1.entry[1])/(v0v1.entry[1]-v2v3.entry[1]);
	}

	/*obtain the position*/
	x = bilinear_interpolate(a, b, v00->x, v01->x, v10->x, v11->x);
	y = bilinear_interpolate(a, b, v00->y, v01->y, v10->y, v11->y);
}


void GlView::compute_a_alongx_degptlocate(double &a, icVector2 v0, icVector2 v1, icVector2 v2, icVector2 v3,
								  icVector2 &v0v1, icVector2 &v2v3)
{
	/*use binary search*/
	/*initialization*/
	double a0, a1;
	bool orient = false;  //false -- CCW, true -- CW
	a0=0.; a1=1.;
	a = 0.5;
	v0v1 = 0.5*(v0+v1);
	v2v3 = 0.5*(v3+v2);
	double theta1, theta2, theta;
	double theta_v0, theta_v1, theta_v2, theta_v3, theta_v03;
	theta_v0 = atan2(v0.entry[1], v0.entry[0]);
	//if(theta_v0<0) theta_v0 += 2*M_PI;
	//theta_v1 = atan2(v1.entry[1], v1.entry[0]);
	//if(theta_v1<0) theta_v1 += 2*M_PI;
	//theta_v2 = atan2(v2.entry[1], v2.entry[0]);
	//if(theta_v2<0) theta_v2 += 2*M_PI;
	theta_v3 = atan2(v3.entry[1], v3.entry[0]);
	//if(theta_v3<0) theta_v3 += 2*M_PI;

	theta_v03 = theta_v3-theta_v0;
	if(theta_v03>=0)
		orient = false; //CCW
	else
		orient = true;
	if(theta_v03<-M_PI) orient = false; //CCW
	if(theta_v03>M_PI) orient = true;   //CW

	/*NOTE: we interpolate from v0 to v1
	                       from v3 to v2 */
	//normalize(v0v1);
	//normalize(v2v3);
	theta1 = atan2(v0v1.entry[1], v0v1.entry[0]); //obtain angle for v0v1;
	//if(theta1<0) theta1 += 2*M_PI;
	theta2 = atan2(v2v3.entry[1], v2v3.entry[0]); //obtain angle for v2v3;
	//if(theta2<0) theta2 += 2*M_PI;
	/*subtract the two angles*/
	theta = theta1-theta2;

	bool s_orient = false;
	while(fabs(fabs(theta)-M_PI)>1e-8 && fabs(a0-a1)>1e-9) /*if they are not opposite to each other*/
	{
		if(theta>=0)
			s_orient = false;                   //CCW
		else
			s_orient = true;                    //CW

		if(theta>M_PI) s_orient = true;         //CW
		if(theta<-M_PI) s_orient = false;       //CCW

		/*if they have the same orientation*/
		if((orient&&s_orient) || (!orient&&!s_orient))
		{
			/*we need to increase a*/
			a1 = a;
			a = (a0+a1)/2.;
		}
		else
		{
			/*we need to decrease a*/
			a0 = a;
			a = (a0+a1)/2.;
		}

		/*recalculate v0v1 and v2v3*/
		v0v1 = (1-a)*v0+a*v1;
		v2v3 = (1-a)*v3+a*v2;

		/*recompute the angles of v0v1 and v2v3*/
		theta1 = atan2(v0v1.entry[1], v0v1.entry[0]); //obtain angle for v0v1;
		if(theta1<0) theta1 += 2*M_PI;
		theta2 = atan2(v2v3.entry[1], v2v3.entry[0]); //obtain angle for v2v3;
		if(theta2<0) theta2 += 2*M_PI;
		/*subtract the two angles*/
		theta = theta1-theta2;

	}
	
	v0v1 = (1-a)*v0+a*v1;
	v2v3 = (1-a)*v3+a*v2;
}
void GlView::display_major_tenlines(GLenum mode)
{
	if(major_path == NULL)
		return;
	int i, j, k;
	Trajectory *cur_traj;
    glColor3f(1, 1, 0);
	glLineWidth(1.5);
	for(j=0; j<major_path->evenstreamlines->ntrajs; j++)
	{
		cur_traj = major_path->evenstreamlines->trajs[j];

		for(k=0; k<cur_traj->nlinesegs; k++)
		{
			glBegin(GL_LINES);
			glVertex2f(cur_traj->linesegs[k].gstart[0], cur_traj->linesegs[k].gstart[1]);
			glVertex2f(cur_traj->linesegs[k].gend[0], cur_traj->linesegs[k].gend[1]);
			glEnd();
		}
	}

}
void GlView::reset_major_path(){
    major_path->reset();
}
void GlView::set_robot_loc(double x,double y){
    robot_loc->pos[0]=x;
    robot_loc->pos[1]=y;
   // reset_major_path();
}
void GlView::gen_major_path(bool type){
	double startpt[2];
	int startcell;
	int fieldtype;
	if(!type) /*major direction*/
		fieldtype=0;
	else
		fieldtype=1;
    robot_loc->triangle=major_path->get_cellID_givencoords(robot_loc->pos[0],robot_loc->pos[1]);
    startcell=robot_loc->triangle;
    startpt[0]=robot_loc->pos[0];
    startpt[1]=robot_loc->pos[1];
	major_path->grow_a_majRoad(startpt, startcell, major_path->percentage_dsep*major_path->dsep, 
		major_path->discsize, major_path->sample_interval, major_path->loopdsep, major_path->dist2sing, 
        major_path->streamlinelength, fieldtype, false,m_robotDirect);
}
void GlView::test(){
//	Seed *seed=new Seed();
//    seed->pos[0]=0.5,seed->pos[1]=0.5;
    gen_major_path(false);
}
void GlView::listen_to_robot_loc(){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("odom", "base_link",
                                  now, ros::Duration(3.0));
      listener.lookupTransform("odom", "base_link",
                               now, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
   //   ros::Duration(1.0).sleep();
    }
    double loc_x=transform.getOrigin().x()/realWorld_to_field_scale+realWorld_to_field_offset_x;
    double loc_y=transform.getOrigin().y()/realWorld_to_field_scale+realWorld_to_field_offset_y;
    set_robot_loc(loc_x,loc_y);
    double x = transform.getRotation().getX();
    double y = transform.getRotation().getY();
    double z = transform.getRotation().getZ();
    double w = transform.getRotation().getW();
    double robot_angel = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    m_robotDirect.entry[0]=cos(robot_angel);
    m_robotDirect.entry[1]=sin(robot_angel);
}
//////////////////////////////////////////////////////////////////////////

void GlView::cancelPath(){
    std_msgs::String msg;
    int count = 0;
    std::stringstream ss;
    ss << "cancel" << count;
    cancel_pub.publish(msg);

    while (count<3)
    {
      cancel_pub.publish(msg);
      ros::spinOnce();
      ++count;
    }
}
void GlView::addRobotWayPoint()
{
    nav_msgs::Path path;
    path.header.frame_id="/odom";

    Trajectory *cur_traj=major_path->evenstreamlines->trajs[0];
    for (int i=0;i<cur_traj->nlinesegs;i++)
    {
         geometry_msgs::PoseStamped pose;
         pose.header.stamp = ros::Time::now();
         double x=(cur_traj->linesegs[i].gend[0]-realWorld_to_field_offset_x)*realWorld_to_field_scale;
         double y=(cur_traj->linesegs[i].gend[1]-realWorld_to_field_offset_y)*realWorld_to_field_scale;
         pose.pose.position.x=x;
         pose.pose.position.y=y;
         pose.header.frame_id="/odom";
         pose.pose.position.z=0;
         pose.pose.orientation.x=0;
         pose.pose.orientation.y=0;
         pose.pose.orientation.z=0;
         pose.pose.orientation.w=1;
         path.poses.push_back(pose);
    }
    int count=0;
    while(count<3)
    {
        count++;
        pub_plan.publish(path);
        ros::spinOnce();
    }
}

void GlView::setDegenerateElem(double center_x,double center_y,int type){
    addto(center_x, center_y,	get_cellID_givencoords(center_x,center_y), type);
    cal_tensorvals_quad_inReg();
    cal_all_eigenvecs_quad();
    init_degpts();
    /*render alpha map*/
    render_alpha_map_quad(false);
    render_alpha_map_quad(true);

    locate_degpts_cells_tranvec_quad();
}
int GlView::get_cellID_givencoords(double x, double y)
{
    int i=(x-quadmesh->xstart)/quadmesh->xinterval;
    int j=(y-quadmesh->ystart)/quadmesh->yinterval;

    //if(i<0) i=0;
    //if(j<0) j=0;

    if(i==quadmesh->XDIM-1) i=quadmesh->XDIM-2;
    if(j==quadmesh->YDIM-1) j=quadmesh->YDIM-2;

    return (j*(quadmesh->XDIM-1)+i);
}


void GlView::addto(double x, double y, int triangle, int type)
{
    /*extend space if not enough*/
    if(ntenelems >= curMaxNumTenDesignElems)
    {
        ten_designelems = (Degenerate_Design*)realloc(ten_designelems, sizeof(Degenerate_Design)*
            (curMaxNumTenDesignElems+20));
        if(ten_designelems == NULL)
            exit(-1);
        curMaxNumTenDesignElems += 20;
    }

    ten_designelems[ntenelems].centerx = x;
    ten_designelems[ntenelems].centery = y;
    ten_designelems[ntenelems].ID = ntenelems;
    ten_designelems[ntenelems].Triangle_ID = triangle;
    ten_designelems[ntenelems].type = type;
    ten_designelems[ntenelems].transform_matrix.setIdentity();

    ten_designelems[ntenelems].rotang = 0;
    ten_designelems[ntenelems].s =
        ten_designelems[ntenelems].sx =
        ten_designelems[ntenelems].sy = 1;
    ten_designelems[ntenelems].deleted = false;

    /*mark which region the design element belongs to 11/21/2007*/
    ten_designelems[ntenelems].which_region=get_region_id(x, y);

    /*  may be a bug 1/18/2008 */
    cur_chosen_region = ten_designelems[ntenelems].which_region;

    //Add the editbox for properties editing
    init_tenelem_EditBox(ntenelems, x, y);

    ntenelems++;
}
void GlView::init_tenelem_EditBox(int index, double x, double y)
{
    ////initial the original edit box
    ten_designelems[index].editbox.p1.entry[0] = x - EDITBOXSIZE;  ////low left point
    ten_designelems[index].editbox.p1.entry[1] = y - EDITBOXSIZE;

    ten_designelems[index].editbox.p2.entry[0] = x - EDITBOXSIZE;  ////upper left point
    ten_designelems[index].editbox.p2.entry[1] = y + EDITBOXSIZE;

    ten_designelems[index].editbox.p3.entry[0] = x + EDITBOXSIZE;  ////upper right point
    ten_designelems[index].editbox.p3.entry[1] = y + EDITBOXSIZE;

    ten_designelems[index].editbox.p4.entry[0] = x + EDITBOXSIZE;  ////low right point
    ten_designelems[index].editbox.p4.entry[1] = y - EDITBOXSIZE;

    ten_designelems[index].editbox.Up.entry[0] = x;  ////rotation controling point
    ten_designelems[index].editbox.Up.entry[1] = y + 2*EDITBOXSIZE;


    ////initial current editbox
    ten_designelems[index].cur_editbox.p1.entry[0] = x - EDITBOXSIZE;  ////low left point
    ten_designelems[index].cur_editbox.p1.entry[1] = y - EDITBOXSIZE;

    ten_designelems[index].cur_editbox.p2.entry[0] = x - EDITBOXSIZE;  ////upper left point
    ten_designelems[index].cur_editbox.p2.entry[1] = y + EDITBOXSIZE;

    ten_designelems[index].cur_editbox.p3.entry[0] = x + EDITBOXSIZE;  ////upper right point
    ten_designelems[index].cur_editbox.p3.entry[1] = y + EDITBOXSIZE;

    ten_designelems[index].cur_editbox.p4.entry[0] = x + EDITBOXSIZE;  ////low right point
    ten_designelems[index].cur_editbox.p4.entry[1] = y - EDITBOXSIZE;

    ten_designelems[index].cur_editbox.Up.entry[0] = x;  ////rotation controling point
    ten_designelems[index].cur_editbox.Up.entry[1] = y + 2*EDITBOXSIZE;
}


void GlView::obstacleCallback(const octomap_fetch::Obstacle &msg){
    obstacle.obstacle.clear();
    obstacle.obstacle=msg.obstacle;
   // ROS_INFO("receive obstacle message, size is: %d",obstacle.obstacle.size());
   //  ROS_INFO("receive obstacle message, size is: %f",obstacle.obstacle[10]);
}
void GlView::targetCallback(const octomap_fetch::Target &msg){
    cur_target=msg;
    cur_target.target_x=cur_target.target_x/realWorld_to_field_scale+realWorld_to_field_offset_x;
    cur_target.target_y=cur_target.target_y/realWorld_to_field_scale+realWorld_to_field_offset_y;
}
void GlView::dirMapCallback(const octomap_fetch::Map &msg){
    dirMap.data.clear();
    dirMap=msg;
    getObstacleContour();
 //   ROS_INFO("receive dirMap message, size is: %d",dirMap.data.size());
   //  ROS_INFO("receive obstacle message, size is: %f",obstacle.obstacle[10]);
}
void GlView::setDriveForce(){
    listen_to_robot_loc();
    set_ten_regBasis(cur_target.target_x,cur_target.target_y,0);
    icVector2 Direct(cur_target.target_x-robot_loc->pos[0],cur_target.target_y-robot_loc->pos[1]);
    normalize(Direct);
    set_ten_regDir(Direct.entry[0]*0.05+cur_target.target_x,Direct.entry[1]*0.05+cur_target.target_y);

//    set_ten_regBasis(robot_loc->pos[0],robot_loc->pos[1],0);
//    icVector2 Direct(cur_target.target_x-robot_loc->pos[0],cur_target.target_y-robot_loc->pos[1]);
//    normalize(Direct);
//    set_ten_regDir(Direct.entry[0]*0.05+robot_loc->pos[0],Direct.entry[1]*0.05+robot_loc->pos[1]);
   // setDegenerateElem(cur_target.target_x,cur_target.target_y,2);
    //set_ten_regDir(contours2World[i][j+1].x,contours2World[i][j+1].y);
}
void GlView::setObstacleAndTarget(){
    resetRegularAndDegenrateElem();
    cancelPath();
    reset_major_path();
    //setDriveForce();
    for(int i=0;i<contours2World.size();i++)
    {  // ROS_INFO("i is ok");
        for (int j=0;j<contours2World[i].size()-1;j++)
        {
            set_ten_regBasis(contours2World[i][j].x, contours2World[i][j].y,0);
         //   ROS_INFO("Point coord is: %f, %f: ",dirMap2World[i][j].x,dirMap2World[i][j].y);
           // icVector2 Direct(contours2World[i][j+1].x-contours2World[i][j].x,contours2World[i][j+1].y-contours2World[i][j].y);
           // normalize(Direct);
            set_ten_regDir(contours2World[i][j+1].x,contours2World[i][j+1].y);
        }
    }
    //ROS_INFO("cur_target.target_x and cur_target.target_y are: %f, %f",cur_target.target_x,cur_target.target_y);
    addto(cur_target.target_x, cur_target.target_y,get_cellID_givencoords(cur_target.target_x,cur_target.target_y), 2);
    cal_tensorvals_quad_inReg();
    cal_all_eigenvecs_quad();

    init_degpts();

    /*calculate the alpha map here*/
    render_alpha_map_quad(false);
    render_alpha_map_quad(true);

    locate_degpts_cells_tranvec_quad();

    listen_to_robot_loc();
    test();
    addRobotWayPoint();
}
void GlView::drawDirMap(){
    //ROS_INFO("drawDirMap");
    glColor3f(1, 0, 0);
    glLineWidth(0.5);
    for(int i=0;i<contours2World.size();i++)
    {  // ROS_INFO("i is ok");
        for (int j=0;j<contours2World[i].size()-1;j++)
        {
            glBegin(GL_LINES);
            glVertex2f(contours2World[i][j].x, contours2World[i][j].y);
         //   ROS_INFO("Point coord is: %f, %f: ",dirMap2World[i][j].x,dirMap2World[i][j].y);
            glVertex2f(contours2World[i][j+1].x, contours2World[i][j+1].y);
            glEnd();
        }
    }

}
void GlView::getObstacleContour(){
      cv::Mat img = cv::Mat::zeros(dirMap.height, dirMap.width, CV_8UC1);
      dirMap2World.clear();
   //   ROS_INFO("height and width is: %d, %d: ",dirMap.height, dirMap.width);
   //   ROS_INFO("origin x and y, and res is: %f, %f, %f: ",dirMap.origin_x, dirMap.origin_y,dirMap.res);
      for (unsigned i=0;i<dirMap.height;i++)
      {
          std::vector<cv::Point2f> tmp;
          for(unsigned j=0;j<dirMap.width;j++)
          {
              cv::Point2f tmp_point;
              tmp_point.x=dirMap.res*(j+0.5)+dirMap.origin_x;
              tmp_point.y=dirMap.res*(dirMap.height-1-i+0.5)+dirMap.origin_y;
              tmp_point.x=tmp_point.x/realWorld_to_field_scale+realWorld_to_field_offset_x;
              tmp_point.y=tmp_point.y/realWorld_to_field_scale+realWorld_to_field_offset_y;
              tmp.push_back(tmp_point);
              img.at<uchar>(i,j)=dirMap.data[dirMap.width*i+j];
          }
          dirMap2World.push_back(tmp);
      }
//      std::vector<std::vector<cv::Point> > contours0;
//      cv::findContours(img, contours0, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//      contours.clear();
//      contours.resize(contours0.size());
//      for( size_t k = 0; k < contours0.size(); k++ )
//          cv::approxPolyDP(cv::Mat(contours0[k]), contours[k], 0.5, true);

      contours.clear();
      cv::findContours(img, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
      m_hull.clear();
      m_hull.resize(contours.size());
      for( int i = 0; i < contours.size(); i++ )
         {  cv::convexHull( cv::Mat(contours[i]), m_hull[i], false ); }

      contours2World.clear();
//      for( size_t k = 0; k < m_hull.size(); k++ )
//      {
//          std::vector<cv::Point2f> tmp_contour;
//          for(size_t i=0;i< m_hull[k].size();i++){
//         //     std::cout<<m_hull[k][i].y<<"  "<<m_hull[k][i].x<<std::endl;
//              tmp_contour.push_back(dirMap2World[m_hull[k][i].y][m_hull[k][i].x]);
//          }
//          contours2World.push_back(tmp_contour);
//         // std::cout<<"next contour"<<std::endl;
//      }
      for( size_t k = 0; k < contours.size(); k++ )
      {
          std::vector<cv::Point2f> tmp_contour;
          for(size_t i=0;i< contours[k].size();i++){
            //  std::cout<<contours[k][i].x<<"  "<<contours[k][i].y<<std::endl;
              tmp_contour.push_back(dirMap2World[contours[k][i].y][contours[k][i].x]);
          }
          contours2World.push_back(tmp_contour);
         // std::cout<<"next contour"<<std::endl;
      }
    //  ROS_INFO("contour size is: %d",contours.size());
      cv::Mat cnt_img = cv::Mat::zeros(dirMap.height, dirMap.width, CV_8UC3);
      cv::drawContours( cnt_img, contours, -1, cv::Scalar(128,255,255),1,8);
      cv::imwrite( "cnt_img.jpg", cnt_img);
}
void GlView::resetRegularAndDegenrateElem(){
    /*initialize regular design element*/
    if (ten_regularelems!=NULL){
        delete(ten_regularelems);
        ten_regularelems=NULL;
        if(ten_regularelems == NULL)
        {
            ten_regularelems=(TenRegularElem *)malloc(sizeof(TenRegularElem)*curMaxNumTenRegElems);
            if(ten_regularelems == NULL)
            {
                ROS_INFO("shan tui le");
                exit(-1);

            }
        }
        nten_regelems = 0;
    }
    /*initialize singular design element*/
    if (ten_designelems!=NULL){
        delete(ten_designelems);
        ten_designelems = NULL;
        if(ten_designelems == NULL)
        {
            ten_designelems=(Degenerate_Design *)malloc(sizeof(Degenerate_Design)*curMaxNumTenDesignElems);
            if(ten_designelems == NULL)
            {
                ROS_INFO("shan tui le");
                exit(-1);
            }
        }
        ntenelems = 0;
    }
}
