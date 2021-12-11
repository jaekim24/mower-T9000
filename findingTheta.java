class findingTheta{
    float Ax,Ay,Bx,By;
    float x, y;

    findingTheta(float x_,float y_){
        //constructor is made to store the x and y points
        //using encapsulation
        x = x_;
        y = y_;
    }

        float mag(){
            //finding length 
            return (float) Math.sqrt((x*x) +( y*y));
        }

        float dot(findingTheta c){
            return (x*c.x) + (y*c.y);
        }


static float findingThetaMethod(float x1,float y1,float x2,float y2){
    //delcaring x and y using encapsulation
    findingTheta a = new findingTheta (x1,y1);
    findingTheta b = new findingTheta (x2,y2);

    // the equation to find the angle between two angles
    // orgin is the thrid point
    // manuplated the dot product to find angle (linear algebra)
    float equation =(float)Math.acos(a.dot(b)/(a.mag()*b.mag())); 
    float theta = (float)Math.toDegrees(equation);

    return theta; 

}

float shiftToOrigin(float x1,float y1,float x2,float y2){
    // need to find third point to be able to shift to origin
    // WORKING ON NOW
    findingTheta a = new findingTheta (x1,y1);
    findingTheta b = new findingTheta (x2,y2);



}


public static void main(String[]arg){
    System.out.println(findingThetaMethod(4,8,8,13));
}
}



