#include "directional_field_to_rviz/Trajectory.h"
Trajectory::Trajectory(int index, int curMaxNum = 200){
	this->index = index;

	if(curMaxNum==0)
	{
		linesegs=NULL;
		curMaxNumLinesegs=0;
		return;
	}

	linesegs = (LineSeg *)malloc(sizeof(LineSeg)*curMaxNum);

	if(linesegs == NULL)
	{
		exit(-1);
	}

	int i;
	for(i=0;i<curMaxNum;i++)
	{
		linesegs[i].gend[0]=linesegs[i].end[0]=linesegs[i].gstart[0]=linesegs[i].start[0]=
			linesegs[i].gend[1]=linesegs[i].end[1]=linesegs[i].gstart[1]=linesegs[i].start[1]=0.;
		linesegs[i].length=0;
		linesegs[i].Triangle_ID=0;
	}

	curMaxNumLinesegs = curMaxNum;
	nlinesegs = 0;
}

Trajectory::~Trajectory()
{
	if(curMaxNumLinesegs > 0)
	{
		free(linesegs);
		curMaxNumLinesegs=0;
	}
}

bool Trajectory::extend_line_segments(int add_size)
{

	LineSeg *extendlist=(LineSeg*)malloc(sizeof(LineSeg)*(curMaxNumLinesegs+add_size));

	if(extendlist == NULL)
		//if(linesegs == NULL)
	{
		return false;
	}

	int i;
	for(i = 0; i < curMaxNumLinesegs; i++)
	{
		extendlist[i].end[0] = linesegs[i].end[0];
		extendlist[i].end[1] = linesegs[i].end[1];

		extendlist[i].start[0] = linesegs[i].start[0];
		extendlist[i].start[1] = linesegs[i].start[1];

		extendlist[i].gend[0] = linesegs[i].gend[0];
		extendlist[i].gend[1] = linesegs[i].gend[1];

		extendlist[i].gstart[0] = linesegs[i].gstart[0];
		extendlist[i].gstart[1] = linesegs[i].gstart[1];

		extendlist[i].length = linesegs[i].length;
		extendlist[i].Triangle_ID = linesegs[i].Triangle_ID;

	}
	free(linesegs);

	linesegs = extendlist;

	for(i=curMaxNumLinesegs;i<curMaxNumLinesegs+add_size;i++)
	{
		linesegs[i].gend[0]=linesegs[i].end[0]=linesegs[i].gstart[0]=linesegs[i].start[0]=
			linesegs[i].gend[1]=linesegs[i].end[1]=linesegs[i].gstart[1]=linesegs[i].start[1]=0.;
		linesegs[i].length=0;
		linesegs[i].Triangle_ID=0;
	}

	curMaxNumLinesegs += add_size;
	return true;
}

/*get the flow length of the streamline*/
double Trajectory::get_length()
{
	int i;
	double len = 0;
	for(i = 0 ; i < nlinesegs; i++)
		len += linesegs[i].length;
	return len;
}


//remove the front n line segments
bool Trajectory::remove_front_nlines(int n)
{
	if(nlinesegs-n<0) return false;
	/*move the content forward*/
	int i;
	for(i=0;i<nlinesegs-n;i++)
	{
		linesegs[i].gstart[0]=linesegs[i+n].gstart[0];
		linesegs[i].gstart[1]=linesegs[i+n].gstart[1];
		linesegs[i].gend[0]=linesegs[i+n].gend[0];
		linesegs[i].gend[1]=linesegs[i+n].gend[1];

		linesegs[i].start[0]=linesegs[i+n].start[0];
		linesegs[i].start[1]=linesegs[i+n].start[1];
		linesegs[i].end[0]=linesegs[i+n].end[0];
		linesegs[i].end[1]=linesegs[i+n].end[1];

		linesegs[i].length=linesegs[i+n].length;
		linesegs[i].Triangle_ID=linesegs[i+n].Triangle_ID;
	}
	nlinesegs-=n;
	return true;
}

//add n new line segments in the front
bool Trajectory::add_front_nlines(LineSeg *otherlinesegs, int n)
{
	if(nlinesegs+n>=curMaxNumLinesegs)
	{
		if(!extend_line_segments(nlinesegs+n-curMaxNumLinesegs))
			exit(-1);
	}
	/*move backward n elements*/
	int i;
	if(nlinesegs>0)
	{
		for(i=nlinesegs-1;i>=0;i--)
		{
			linesegs[i+n].gstart[0]=linesegs[i].gstart[0];
			linesegs[i+n].gstart[1]=linesegs[i].gstart[1];
			linesegs[i+n].gend[0]=linesegs[i].gend[0];
			linesegs[i+n].gend[1]=linesegs[i].gend[1];

			linesegs[i+n].start[0]=linesegs[i].start[0];
			linesegs[i+n].start[1]=linesegs[i].start[1];
			linesegs[i+n].end[0]=linesegs[i].end[0];
			linesegs[i+n].end[1]=linesegs[i].end[1];

			linesegs[i+n].length=linesegs[i].length;
			linesegs[i+n].Triangle_ID=linesegs[i].Triangle_ID;
		}
	}

	/*copy the new n line segments to the front*/
	for(i=0;i<n;i++)
	{
		linesegs[i].gstart[0]=otherlinesegs[i].gstart[0];
		linesegs[i].gstart[1]=otherlinesegs[i].gstart[1];
		linesegs[i].gend[0]=otherlinesegs[i].gend[0];
		linesegs[i].gend[1]=otherlinesegs[i].gend[1];

		linesegs[i].start[0]=otherlinesegs[i].start[0];
		linesegs[i].start[1]=otherlinesegs[i].start[1];
		linesegs[i].end[0]=otherlinesegs[i].end[0];
		linesegs[i].end[1]=otherlinesegs[i].end[1];

		linesegs[i].length=otherlinesegs[i].length;
		linesegs[i].Triangle_ID=otherlinesegs[i].Triangle_ID;
	}
	nlinesegs+=n;
	return true;
}

//remove the last n line segments
bool Trajectory::remove_last_nlines(int n)
{
	if(nlinesegs-n<0) return false;
	nlinesegs-=n;
	return true;
}

//add n new line segments at the end
bool Trajectory::add_last_nlines(LineSeg *otherlinesegs, int n)
{
	if(nlinesegs+n>=curMaxNumLinesegs)
	{
		if(!extend_line_segments(nlinesegs+n-curMaxNumLinesegs))
			exit(-1);
	}

	/*copy the content of "linesegs" to the end of current list*/
	int i;
	for(i=nlinesegs;i<n+nlinesegs;i++)
	{
		linesegs[i].gstart[0]=otherlinesegs[i-nlinesegs].gstart[0];
		linesegs[i].gstart[1]=otherlinesegs[i-nlinesegs].gstart[1];
		linesegs[i].gend[0]=otherlinesegs[i-nlinesegs].gend[0];
		linesegs[i].gend[1]=otherlinesegs[i-nlinesegs].gend[1];

		linesegs[i].start[0]=otherlinesegs[i-nlinesegs].start[0];
		linesegs[i].start[1]=otherlinesegs[i-nlinesegs].start[1];
		linesegs[i].end[0]=otherlinesegs[i-nlinesegs].end[0];
		linesegs[i].end[1]=otherlinesegs[i-nlinesegs].end[1];

		linesegs[i].length=otherlinesegs[i-nlinesegs].length;
		linesegs[i].Triangle_ID=otherlinesegs[i-nlinesegs].Triangle_ID;
	}
	nlinesegs+=n;
	return true;
}

bool Trajectory::store_to_global_line_segs(CurvePoints *temp, int num)
{
	int i;
	int tempid = nlinesegs;
	icVector3 dis_vec;

	////if the number of the line segements over the maximum number of the line segments each trajectory can store
	////extend the space for each trajectory
	if(tempid + num - 1 >= curMaxNumLinesegs)
	{
		//if(curMaxNumLinesegs>1000) 
		//	return false; // possible bug here! 12/27/2007

		if(!extend_line_segments(200))
		{
			return false;
		}
	}

	/*save to the global list*/

	for( i = 0; i < num-1; i++)
	{
		////Build the line segment
		linesegs[tempid+i].gstart[0] = temp[i].gpx;
		linesegs[tempid+i].gstart[1] = temp[i].gpy;
		linesegs[tempid+i].start[0] = temp[i].lpx;
		linesegs[tempid+i].start[1] = temp[i].lpy;

		linesegs[tempid+i].gend[0] = temp[i+1].gpx;
		linesegs[tempid+i].gend[1] = temp[i+1].gpy;
		linesegs[tempid+i].end[0] = temp[i+1].lpx;
		linesegs[tempid+i].end[1] = temp[i+1].lpy;

		////Use local coordinates to calculate the length
		dis_vec.entry[0] = temp[i+1].gpx - temp[i].gpx;
		dis_vec.entry[1] = temp[i+1].gpy - temp[i].gpy;
		dis_vec.entry[2] = 0;

		linesegs[tempid+i].length = length(dis_vec);

		linesegs[tempid+i].Triangle_ID = temp[i].triangleid;
	}

	nlinesegs = tempid + num - 1;
	return true;
}

//bool Trajectory::reverse_lines()
//{
//	int i;
//
//	LineSeg *temp = (LineSeg *)malloc(sizeof(LineSeg)*(this->nlinesegs+1));
//
//	if(temp == NULL)
//	{
//		return false;
//	}
//
//	int newnum_lines = 0;
//
//	//store the line segment in reversed order
//
//	for(i = this->nlinesegs-1; i >= 0; i--)
//	{
//		if(this->linesegs[i].Triangle_ID < 0
//			|| this->linesegs[i].Triangle_ID >= quadmesh->nfaces  
//			|| this->linesegs[i].length < 0)
//		{
//			continue;
//		}
//
//		temp[newnum_lines].gstart[0] = this->linesegs[i].gend[0];
//		temp[newnum_lines].gstart[1] = this->linesegs[i].gend[1];
//
//		temp[newnum_lines].gend[0] = this->linesegs[i].gstart[0];
//		temp[newnum_lines].gend[1] = this->linesegs[i].gstart[1];
//
//		temp[newnum_lines].start[0] = this->linesegs[i].end[0];
//		temp[newnum_lines].start[1] = this->linesegs[i].end[1];
//
//		temp[newnum_lines].end[0] = this->linesegs[i].start[0];
//		temp[newnum_lines].end[1] = this->linesegs[i].start[1];
//
//		temp[newnum_lines].length = this->linesegs[i].length;
//		temp[newnum_lines].Triangle_ID = this->linesegs[i].Triangle_ID;
//
//
//		newnum_lines++;
//	}
//
//	//Copy it back to the origin array
//	for(i = 0; i < newnum_lines; i++)
//	{
//		this->linesegs[i].gstart[0] = temp[i].gstart[0];
//		this->linesegs[i].gstart[1] = temp[i].gstart[1];
//
//		this->linesegs[i].gend[0] = temp[i].gend[0];
//		this->linesegs[i].gend[1] = temp[i].gend[1];
//
//		this->linesegs[i].start[0] = temp[i].start[0];
//		this->linesegs[i].start[1] = temp[i].start[1];
//
//		this->linesegs[i].end[0] = temp[i].end[0];
//		this->linesegs[i].end[1] = temp[i].end[1];
//
//		this->linesegs[i].length = temp[i].length;
//		this->linesegs[i].Triangle_ID = temp[i].Triangle_ID;
//	}
//
//	this->nlinesegs = newnum_lines;
//
//	free(temp);
//}

//int Trajectory::trace_in_quad(int &face_id, double globalp[2], int type, int &flag)
//{
//
//	int i;
//	double pre_point[2];
//
//	/*  will this be a good solution? 1/9/2008 */
//	if(!is_in_cell(face_id, globalp[0], globalp[1]))
//	{
//		face_id = get_cellID_givencoords(globalp[0], globalp[1]);
//	}
//
//	if(face_id < 0 || face_id>=quadmesh->nfaces)
//		return -1;
//
//	QuadCell *face = quadmesh->quadcells[face_id];
//
//	QuadCell *pre_f = face;
//
//	Temporary curve point array
//
//	CurvePoints *temp_point_list = (CurvePoints*) malloc(sizeof(CurvePoints) * 200);
//
//	if(temp_point_list == NULL)
//	{
//		exit(-1);
//	}
//
//	int NumPoints = 0;
//
//	/*the tracing will be performed under the global frame*/
//	globalface = face_id;
//
//	pre_point[0] = globalp[0];
//	pre_point[1] = globalp[1];
//
//
//	////////////////////////////////////////////////
//	for(i = 0; i < 200; i++)
//	{
//
//		2. if current point is inside current triangle
//		if(is_in_cell(face_id, globalp[0], globalp[1]))
//		{
//			store the point into the temp curve points list
//
//			temp_point_list[NumPoints].gpx = globalp[0];
//			temp_point_list[NumPoints].gpy = globalp[1];
//			temp_point_list[NumPoints].triangleid = face->index;  
//			NumPoints++;
//
//			pre_point[0] = globalp[0];
//			pre_point[1] = globalp[1];
//
//			/*change to use other integration scheme 07/09/07*/
//			if(compute_next_pt_tensor_quad_global(pre_point, globalp, face_id))
//			if(get_nextpt_2ndeuler_ten_quad(pre_point, globalp, face_id, type))
//				if(get_nextpt_RK23_ten_quad(pre_point, globalp, face_id, type))
//					if(get_nextpt_RK45_ten_quad(pre_point, globalp, face_id, type))
//			{
//				/*obtain the global direction of current tensor line 09/20/2007*/
//				tenline_dir_global_p = tenline_dir_global;
//
//				tenline_dir_global.entry[0] = globalp[0] - pre_point[0];
//				tenline_dir_global.entry[1] = globalp[1] - pre_point[1];
//
//			}
//
//			else{  ////the curve reach a singularity/degenerate point
//				flag = 1;
//
//				Store the record into global line segment array
//
//				if(store_to_global_line_segs(temp_point_list, NumPoints))
//				{
//					Not enough memory
//					flag = 4;
//					free(temp_point_list);
//					return face_id;
//				}
//
//				free(temp_point_list);
//
//				return face_id;
//			}
//		}
//
//
//		3. if the point is out of current cell
//		else{
//
//			/*!!!!!!need to judge which cell it will enter!!!!!*/
//			int PassVertornot = 0;
//
//			get_next_cell_2(face_id, pre_point, globalp, PassVertornot, type);
//
//			if(PassVertornot>0)  /*cross a vertex*/
//			{
//				/*obtain the global direction of current tensor line 09/20/2007*/
//				tenline_dir_global_p = tenline_dir_global;
//
//				tenline_dir_global.entry[0] = pre_point[0] - globalp[0];
//				tenline_dir_global.entry[1] = pre_point[1] - globalp[1];
//
//				/**/
//				temp_point_list[NumPoints].gpx = globalp[0];
//				temp_point_list[NumPoints].gpy = globalp[1];
//				temp_point_list[NumPoints].triangleid = face->index;  ////cause problem 05/25/05
//				NumPoints++;
//
//				temp_point_list[NumPoints].gpx = pre_point[0];
//				temp_point_list[NumPoints].gpy = pre_point[1];
//				temp_point_list[NumPoints].triangleid = face_id;  ////cause problem 05/25/05
//				NumPoints++;
//			}
//			else{
//				/*obtain the global direction of current tensor line 09/20/2007*/
//				tenline_dir_global.entry[0] = globalp[0] - pre_point[0];
//				tenline_dir_global.entry[1] = globalp[1] - pre_point[1];
//
//				Add the intersection point to the temporary points' list
//				temp_point_list[NumPoints].gpx = globalp[0];
//				temp_point_list[NumPoints].gpy = globalp[1];
//				temp_point_list[NumPoints].triangleid = face->index;  ////cause problem 05/25/05
//				NumPoints++;
//			}
//
//			if(face_id<0||face_id>=quadmesh->nfaces)
//				{
//					int test=0;
//				}
//
//			if(NumPoints > 1){
//				Store the record into global line segment array
//				if(!store_to_global_line_segs(temp_point_list, NumPoints))
//				{   ////Not enough memory
//					flag = 4;
//					free(temp_point_list);
//					return face_id;
//				}
//			}
//
//			free(temp_point_list);
//			return face_id;
//		}
//
//	}
//
//	if(NumPoints > 0)
//		if(!store_to_global_line_segs(temp_point_list, NumPoints))
//			flag=4;
//
//	free(temp_point_list);
//
//	return face_id;
//}







