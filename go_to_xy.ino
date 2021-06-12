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

int find_distance_of_2_coordinates(int x1,int x2,int y1,int y2){
    int distance ;

    distance = sqrt((x2-x1)^2 + (y2-y1)^2);

    return distance;
}

float find_angle(){
    //for right now insertting the target coordinates

    float angle;

    angle = acos(find_distance_of_2_coordinates(0,10,0,0)/find_distance_of_2_coordinates(0,10,0,5));

    return angle;
}

void go_to_xy(){
    // mower is at (0,0) and target is (10,5)
    x_now = 0;
    y_now = 0;
    x_target = 10;
    y_target = 5;

    //how to you want to set the z as 
    if(z_now >z_target){

    }    


}

void main(){
}









