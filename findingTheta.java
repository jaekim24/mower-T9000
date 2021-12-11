class findingTheta{
    float Ax,Ay,Bx,By;
    float x, y;

    findingTheta(float x_,float y_){
        x = x_;
        y = y_;
    }

        float mag(){
            return (float) Math.sqrt((x*x) +( y*y));
        }

        float dot(findingTheta c){
            return (x*c.x) + (y*c.y);
        }

        void debug(findingTheta m){
            System.out.println( x + ","+ y + ","+ m.x+","+m.y);
        }

float dot(float AX,float AY,float BX,float BY){
    return ((AX*BX)+(AY*BY));
} 

static float findingThetaMethod(float x1,float y1,float x2,float y2){
    findingTheta a = new findingTheta (x1,y1);
    findingTheta b = new findingTheta (x2,y2);
    float equation =(float)Math.acos(a.dot(b)/(a.mag()*b.mag())); 
    float theta = (float)Math.toDegrees(equation);

    return theta; 

}
public static void main(String[]arg){
    System.out.println(findingThetaMethod(4,8,8,13));
}
}

