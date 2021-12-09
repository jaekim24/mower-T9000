import java.lang.*;
class findingThetaV2{
    float Ax,Ay,Bx,By;
    float x, y;

    findingThetaV2(float x_,float y_){
        x = x_;
        y = y_;
    }

        float mag(findingThetaV2 v){
            return (float) Math.sqrt(x*v.x + y*v.y);
        }

        float dot(findingThetaV2 c){
            return x*c.x + y*c.y;
        }

/*
        float getAx(){
            return Ax_;
        }
        float getAy(){
            return Ay_;
        }
        float getBx(){
            return Bx_;
        }
        float getBy(){
            return By_;
        }
        
        void setAx(float x){
            Ax = x ; 
        }
        void setAy(float y){
            Ay = y;
        }
        void setBx(float x){
            Bx = x;
        }
        void setBy(float y){
            By = y;
        }
*/
float dot(float AX,float AY,float BX,float BY){
    return ((AX*BX)+(AY*BY));
} 

static float findingTheta(float x1,float y1,float x2,float y2){
    findingThetaV2 a = new findingThetaV2 (x1,y1);
    findingThetaV2 b = new findingThetaV2 (x2,y2);

    float theta = (float)Math.acos(a.dot(b)/(a.mag(b)+b.mag(a)));

    return theta; 

}
public static void main(String[]arg){
     findingTheta(10,2,4,-3);
   
}
}


