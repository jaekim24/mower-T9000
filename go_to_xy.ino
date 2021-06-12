#include <math.h>
#include <stdio.h> 
#include <cmath>
/*
find angle a == angle the mower has to turn to go to target xy 
                    b (10,5)

a (0,0)             c (10,0)

distance formula : d = sqrt((x2-x1)^2 + (y2-y1)^2)

-issues: >going to xy at the wrong orientation 
         >how do you move to the xy coordinates 

-solution: >if x_target > x_now then turn to right( but by how much? )
*/

int x_target,y_target,x_now,y_now,z_now,z_target;
float new_z;


int find_distance_of_2_coordinates(int x1,int y1,int x2,int y2){
    int distance ;
    distance = sqrt((x2-x1)^2 + (y2-y1)^2);
    return distance;
}


float find_angle(int x1,int y1,int x2,int y2,int x3,int y3 ){
    //                                      (x2,y2) <----target
    // so the angle is at (x1,y1)           (x3,y3)
    float angle;
    angle = acos(find_distance_of_2_coordinates(x1,y1,x2,y2)/find_distance_of_2_coordinates(x1,y1,x3,y3));
    return angle;
}


float get_z(){
    new_z = z - 256; 
    return (new_z);
}


void go_to_xy(int x1,int y1,int x2,int y2,int x3,int y3){
    // mower is at (0,0) and target is (10,5)
    x_now = 0;  // modify slave nano to receive the xy  
    y_now = 0;  // same here ^
    x_target = x2;
    y_target = y2;
    z_target = find_angle( x1, y1, x2, y2, x3,y3);

    //how to you want to set the z as 
    while (z_target != get_z()){
        if(get_z() >z_target){
            while(get_z()> z_target){
                ST.motor(1,60); //turns right, moving the left motor
            }
            ST.motor(1,0); //stops once z_now == z_target
        }    
        if(get_z() < z_target){
            while(get_z() < z_target){
                ST.motor(2,60); // turns left, moving the right motor
            }
            ST.motor(2,0);
        }
    }
}









