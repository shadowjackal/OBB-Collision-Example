/*
** Jo Sega Saturn Engine
** Copyright (c) 2012-2021, Johannes Fetz (johannesfetz@gmail.com)
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Johannes Fetz nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURposE ARE
** DISCLAIMED. IN NO EVENT SHALL Johannes Fetz BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE posSIBILITY OF SUCH DAMAGE.
*/

#include <jo/jo.h>

jo_camera               cam;
MATRIX matrix;
int colliding;

typedef struct vector3{
    jo_fixed x,y,z;
}   Vec3;

int mode;

Vec3    overlapAxis;
jo_fixed    overlapVal;

int eulerx;
int eulery;
int eulerz;

typedef struct obbbb{
    Vec3 pos,xaxis,yaxis,zaxis,Half_size,spd;
    jo_3d_mesh *jo_mesh;
    Vec3 s[24];
}   OBB;

jo_fixed    dotv(Vec3 v1, Vec3 v2) {
    return (jo_fixed_mult(v1.x , v2.x) + jo_fixed_mult(v1.y , v2.y) + jo_fixed_mult(v1.z , v2.z));
}

Vec3        crossv(Vec3 v1,Vec3 v2) {
    Vec3    rvec;
    rvec.x = (jo_fixed_mult(v1.y , v2.z) - jo_fixed_mult(v1.z , v2.y));
    rvec.y = (jo_fixed_mult(v1.z , v2.x) - jo_fixed_mult(v1.x , v2.z));
    rvec.z = (jo_fixed_mult(v1.x , v2.y) - jo_fixed_mult(v1.y , v2.x));
    return rvec;
};

Vec3        addv(Vec3 v1, Vec3 v2) {
    Vec3 rvec;
    rvec.x = v1.x + v2.x;
    rvec.y = v1.y + v2.y;
    rvec.z = v1.z + v2.z;
    return rvec;
}

Vec3        subv(Vec3 v1, Vec3 v2) { // subtracts two vectors and returns a new one
    Vec3 rvec;
    rvec.x = v1.x - v2.x;
    rvec.y = v1.y - v2.y;
    rvec.z = v1.z - v2.z;
    return rvec;
}

Vec3        idkv(Vec3 v1, jo_fixed var) {
    Vec3 rvec;
    rvec.x = jo_fixed_mult(v1.x, var); 
    rvec.y = jo_fixed_mult(v1.y, var);
    rvec.z = jo_fixed_mult(v1.z, var);
    return rvec;
}

jo_fixed calc(Vec3 Plane, Vec3 a, jo_fixed b) {
  return JO_ABS(dotv(idkv(a,b), Plane));
}

/*bool getSepPlane(Vec3 Rpos, Vec3 Plane,  OBB *box1, OBB *box2) {
  OBB* boxes[2] = {box1, box2};
  int rval = 0;
  for(int i=0; i < 2; ++i){
        rval += calc(Plane, boxes[i]->xaxis,boxes[i]->Half_size.x) +
                  calc(Plane, boxes[i]->yaxis,boxes[i]->Half_size.y) +
                  calc(Plane, boxes[i]->zaxis,boxes[i]->Half_size.z);
  }
     

 if(JO_ABS(dotv(Rpos,Plane)) < rval)  {
      if((dotv(Plane,Rpos)) < rval) {
          overlapVal = JO_ABS(JO_FIXED_1);
          overlapAxis = Plane;
      }}

  return (dotv(Rpos,Plane)) > rval;
}*/


bool getSepPlane(Vec3 Rpos, Vec3 Plane,  OBB *box1, OBB *box2) {
    return (JO_ABS(dotv(Rpos,Plane)) > 
        (JO_ABS(dotv((idkv(box1->xaxis,box1->Half_size.x)),Plane)) +
        JO_ABS(dotv((idkv(box1->yaxis,box1->Half_size.y)),Plane)) +
        JO_ABS(dotv((idkv(box1->zaxis,box1->Half_size.z)),Plane)) +
        JO_ABS(dotv((idkv(box2->xaxis,box2->Half_size.x)),Plane)) + 
        JO_ABS(dotv((idkv(box2->yaxis,box2->Half_size.y)),Plane)) +
        JO_ABS(dotv((idkv(box2->zaxis,box2->Half_size.z)),Plane))));
}


bool getCollision(OBB *box1, OBB *box2) {
    Vec3 Rpos;
    Rpos = subv(box2->pos, box1->pos);

    return !(getSepPlane(Rpos, box1->xaxis, box1, box2) ||
        getSepPlane(Rpos, box1->zaxis, box1, box2) ||
        getSepPlane(Rpos, box1->yaxis, box1, box2) ||
        getSepPlane(Rpos, box2->xaxis, box1, box2) ||
        getSepPlane(Rpos, box2->yaxis, box1, box2) ||
        getSepPlane(Rpos, box2->zaxis, box1, box2) ||
        getSepPlane(Rpos, crossv(box1->xaxis,box2->xaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->xaxis,box2->yaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->xaxis,box2->zaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->yaxis,box2->xaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->yaxis,box2->yaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->yaxis,box2->zaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->zaxis,box2->xaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->zaxis,box2->yaxis), box1, box2) ||
        getSepPlane(Rpos, crossv(box1->zaxis,box2->zaxis), box1, box2));
};


void        InitObb(OBB *obium, int xwidth, int ywidth, int height) {
        obium->Half_size.x = jo_int2fixed(xwidth);
        obium->Half_size.y = jo_int2fixed(height);
        obium->Half_size.z = jo_int2fixed(ywidth);
        obium->xaxis.x = jo_int2fixed(1); obium->xaxis.y = jo_int2fixed(0); obium->xaxis.z = jo_int2fixed(0);
        obium->yaxis.x = jo_int2fixed(0); obium->yaxis.y = jo_int2fixed(1); obium->yaxis.z = jo_int2fixed(0);
        obium->zaxis.x = jo_int2fixed(0); obium->zaxis.y = jo_int2fixed(0); obium->zaxis.z = jo_int2fixed(1);
        obium->jo_mesh = jo_3d_create_mesh(6);

        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(height), jo_int2fixed(ywidth), 0);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(height), jo_int2fixed(ywidth), 1);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(height), jo_int2fixed(-ywidth), 2);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(height), jo_int2fixed(-ywidth), 3);   

        obium->s[0].x = jo_int2fixed(-xwidth);  obium->s[0].y = jo_int2fixed(height); obium->s[0].z = jo_int2fixed(ywidth);
        obium->s[1].x = jo_int2fixed(xwidth);   obium->s[1].y = jo_int2fixed(height); obium->s[1].z = jo_int2fixed(ywidth);
        obium->s[2].x = jo_int2fixed(xwidth);   obium->s[2].y = jo_int2fixed(height); obium->s[2].z = jo_int2fixed(-ywidth);
        obium->s[3].x = jo_int2fixed(-xwidth);  obium->s[3].y = jo_int2fixed(height); obium->s[3].z = jo_int2fixed(-ywidth);   

        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(-height), jo_int2fixed(ywidth), 4);
        jo_3d_set_mesh_vertice(obium->jo_mesh,  jo_int2fixed(xwidth), jo_int2fixed(-height), jo_int2fixed(ywidth), 5);
        jo_3d_set_mesh_vertice(obium->jo_mesh,  jo_int2fixed(xwidth), jo_int2fixed(-height), jo_int2fixed(-ywidth), 6);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(-height), jo_int2fixed(-ywidth), 7);

        obium->s[4].x = jo_int2fixed(-xwidth);  obium->s[4].y = jo_int2fixed(-height); obium->s[4].z = jo_int2fixed(ywidth);
        obium->s[5].x = jo_int2fixed(xwidth);  obium->s[5].y = jo_int2fixed(-height); obium->s[5].z = jo_int2fixed(ywidth);
        obium->s[6].x = jo_int2fixed(xwidth);  obium->s[6].y = jo_int2fixed(-height); obium->s[6].z = jo_int2fixed(-ywidth);
        obium->s[7].x = jo_int2fixed(-xwidth); obium->s[7].y = jo_int2fixed(-height); obium->s[7].z = jo_int2fixed(-ywidth);   

        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(-height),   jo_int2fixed(ywidth), 8);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth),  jo_int2fixed(height),   jo_int2fixed(ywidth), 9);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth),  jo_int2fixed(height), jo_int2fixed(-ywidth), 10);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(-height), jo_int2fixed(-ywidth), 11);

        obium->s[8].x = jo_int2fixed(-xwidth);  obium->s[8].y = jo_int2fixed(-height); obium->s[8].z = jo_int2fixed(ywidth);
        obium->s[9].x = jo_int2fixed(-xwidth);  obium->s[9].y = jo_int2fixed(height); obium->s[9].z = jo_int2fixed(ywidth);
        obium->s[10].x = jo_int2fixed(-xwidth);  obium->s[10].y = jo_int2fixed(height); obium->s[10].z = jo_int2fixed(-ywidth);
        obium->s[11].x = jo_int2fixed(-xwidth); obium->s[11].y = jo_int2fixed(-height); obium->s[11].z = jo_int2fixed(-ywidth);   

        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(-height), jo_int2fixed(ywidth), 12);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(height), jo_int2fixed(ywidth), 13);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(height), jo_int2fixed(-ywidth), 14);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(-height), jo_int2fixed(-ywidth), 15);

        obium->s[12].x = jo_int2fixed(xwidth);  obium->s[12].y = jo_int2fixed(-height); obium->s[12].z = jo_int2fixed(ywidth);
        obium->s[13].x = jo_int2fixed(xwidth);  obium->s[13].y = jo_int2fixed(height); obium->s[13].z = jo_int2fixed(ywidth);
        obium->s[14].x = jo_int2fixed(xwidth);  obium->s[14].y = jo_int2fixed(height); obium->s[14].z = jo_int2fixed(-ywidth);
        obium->s[15].x = jo_int2fixed(xwidth); obium->s[15].y = jo_int2fixed(-height); obium->s[15].z = jo_int2fixed(-ywidth);   
        
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(height), jo_int2fixed(ywidth), 16);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(height), jo_int2fixed(ywidth), 17);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(-height), jo_int2fixed(ywidth), 18);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(-height), jo_int2fixed(ywidth), 19);

        obium->s[16].x = jo_int2fixed(-xwidth);  obium->s[16].y = jo_int2fixed(height); obium->s[16].z = jo_int2fixed(ywidth);
        obium->s[17].x = jo_int2fixed(xwidth);  obium->s[17].y = jo_int2fixed(height); obium->s[17].z = jo_int2fixed(ywidth);
        obium->s[18].x = jo_int2fixed(xwidth);  obium->s[18].y = jo_int2fixed(-height); obium->s[18].z = jo_int2fixed(ywidth);
        obium->s[19].x = jo_int2fixed(-xwidth); obium->s[19].y = jo_int2fixed(-height); obium->s[19].z = jo_int2fixed(ywidth);   

        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(height), jo_int2fixed(-ywidth), 20);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(height), jo_int2fixed(-ywidth), 21);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(xwidth), jo_int2fixed(-height), jo_int2fixed(-ywidth), 22);
        jo_3d_set_mesh_vertice(obium->jo_mesh, jo_int2fixed(-xwidth), jo_int2fixed(-height), jo_int2fixed(-ywidth), 23);

        obium->s[20].x = jo_int2fixed(-xwidth);  obium->s[20].y = jo_int2fixed(height); obium->s[20].z = jo_int2fixed(-ywidth);
        obium->s[21].x = jo_int2fixed(xwidth);  obium->s[21].y = jo_int2fixed(height); obium->s[21].z = jo_int2fixed(-ywidth);
        obium->s[22].x = jo_int2fixed(xwidth);  obium->s[22].y = jo_int2fixed(-height); obium->s[22].z = jo_int2fixed(-ywidth);
        obium->s[23].x = jo_int2fixed(-xwidth); obium->s[23].y = jo_int2fixed(-height); obium->s[23].z = jo_int2fixed(-ywidth);   

        jo_3d_set_mesh_color(obium->jo_mesh, JO_COLOR_Green);
}

void    update_polygon(OBB *a) 
{
    for(int i = 0; i < 24; i+=4) {
        jo_3d_set_mesh_vertice(a->jo_mesh, a->s[i + 0].x,  a->s[i + 0].y , a->s[i + 0].z, i + 0);
        jo_3d_set_mesh_vertice(a->jo_mesh, a->s[i + 1].x,  a->s[i + 1].y , a->s[i + 1].z, i + 1);
        jo_3d_set_mesh_vertice(a->jo_mesh, a->s[i + 2].x,  a->s[i + 2].y , a->s[i + 2].z, i + 2);
        jo_3d_set_mesh_vertice(a->jo_mesh, a->s[i + 3].x,  a->s[i + 3].y , a->s[i + 3].z, i + 3);
    }
}

OBB A;
OBB B;

void			    my_draw(void)
{

    if (jo_is_input_key_pressed(0, JO_KEY_L)) {
        if(mode == 0) eulerx -= 1;
        if(mode == 1) eulery -= 1;
        if(mode == 2) eulerz -= 1;
    }

    if (jo_is_input_key_pressed(0, JO_KEY_R)) {
        if(mode == 0) eulerx += 1;
        if(mode == 1) eulery += 1;
        if(mode == 2) eulerz += 1;
    }
  


    if (jo_is_input_key_pressed(0, JO_KEY_X))
        mode = 0;

    if (jo_is_input_key_pressed(0, JO_KEY_Y))
        mode = 1;

    if (jo_is_input_key_pressed(0, JO_KEY_Z))
        mode = 2;

    if (jo_is_input_key_pressed(0, JO_KEY_A))
        A.pos.y += JO_FIXED_1;

    if (jo_is_input_key_pressed(0, JO_KEY_B))
        A.pos.y -= JO_FIXED_1;



    switch (jo_get_input_direction_pressed(0))
    {
        case LEFT: A.pos.x -= jo_int2fixed(1); break;
        case RIGHT: A.pos.x += jo_int2fixed(1); break;
        case UP: A.pos.z += jo_int2fixed(1); break;
        case DOWN: A.pos.z -= jo_int2fixed(1); break;

        case NONE: A.spd.x = 0; A.spd.y = 0; A.spd.z = 0;break;
    }
    update_polygon(&A);
    update_polygon(&B);
    jo_printf(12, 1, "*Custom 3D demo*");

    jo_3d_push_matrix();
	{
        slGetMatrix(matrix);
        slRotX(jo_DEGtoANG_int(eulerx));
        slRotY(jo_DEGtoANG_int(eulery));
        slRotZ(jo_DEGtoANG_int(eulerz));
        slGetMatrix(matrix);
    }
    jo_3d_pop_matrix();
    A.xaxis.x = matrix[X][X];    
    A.xaxis.y = matrix[X][Y];
    A.xaxis.z = matrix[X][Z];    
    A.yaxis.x = matrix[Y][X];    
    A.yaxis.y = matrix[Y][Y];
    A.yaxis.z = matrix[Y][Z];    
    A.zaxis.x = matrix[Z][X];    
    A.zaxis.y = matrix[Z][Y];
    A.zaxis.z = matrix[Z][Z];    

    if(getCollision(&A,&B))  {
        jo_3d_set_mesh_color(A.jo_mesh, JO_COLOR_Red);
        A.xaxis = B.xaxis;
        A.yaxis = B.yaxis;
        A.zaxis = B.zaxis;
        colliding = 1;
        } else  {
            jo_3d_set_mesh_color(A.jo_mesh, JO_COLOR_Yellow);
            colliding = 0;
        }




    jo_3d_camera_look_at(&cam);
    jo_3d_push_matrix();
    {
        jo_3d_translate_matrix_fixed(A.pos.x,A.pos.y,A.pos.z);
        if(colliding == 0) jo_3d_rotate_matrix(eulerx,eulery,eulerz);

        jo_3d_mesh_draw(A.jo_mesh);
    }
    jo_3d_pop_matrix();

    jo_3d_push_matrix();
    {
        jo_3d_translate_matrix_fixed(B.pos.x,B.pos.y,B.pos.z);
        jo_3d_mesh_draw(B.jo_mesh);
    }
    jo_3d_pop_matrix();
    jo_printf(0, 24, "overlapValue : %d                                 ", jo_fixed2int(overlapVal));
    jo_printf(0, 28, "overlap axis : %d   %d   %d             ", overlapAxis.x,overlapAxis.y,overlapAxis.z);
    overlapVal = JO_FIXED_MAX;
}

void			jo_main(void)
{
	jo_core_init(JO_COLOR_Black);
    jo_3d_camera_init(&cam);

        // create two obbs

    InitObb(&A,25,25,25);
    // set the first obb's properties
    A.pos.x = jo_int2fixed(-48);//, 0, 0}; // set its center position
    A.pos.z = jo_int2fixed(0);
    A.pos.y = jo_int2fixed(64);

    // set the half size

    InitObb(&B,30,30,30);
    // set the second obb's properties
   B.pos.x = jo_int2fixed(20);//, 0, 0}; // set its center position


    // set the axes orientation


	jo_core_add_callback(my_draw);
	jo_core_run();
}

/*
** END OF FILE
*/
