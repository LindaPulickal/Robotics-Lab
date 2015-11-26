#define ARRAY_LEN(X) (sizeof(X) / sizeof((X)[0]))
#define ERROR_SIZE 5

class PIDController {

private:
    float Kp;
    float Kd;
    float Ki;
    float output;
    float derivative;
    float prevErrors[ERROR_SIZE];

    void storeError(float error) {
        for(int i=ARRAY_LEN(prevErrors)-1; i>0; i--) {
            prevErrors[i] = prevErrors[i-1];
        }
        prevErrors[0] = error;
    }

    void showStoredErrors() {
        for(int i=0; i<ERROR_SIZE; i++){
            printf("%f, ",prevErrors[i]);
        }	
	    fflush(stdout);
    }
public:

    PIDController(float Kp_, float Kd_, float Ki_) {
        Kp = Kp_;
        Kd = Kd_;
        Ki = Ki_;
        output = 0;
        derivative = 0;
        for(int i=0; i<ERROR_SIZE; i++){
            prevErrors[i] = 0;
        }
    }

    float getErrorSum() {
        float sum=0;
        for(int i=0; i<ERROR_SIZE; i++)
            sum += prevErrors[i];
        return sum;
    }

    int getPIDValue(float goalDistance, float curDistance) {
        float error = goalDistance - curDistance;
        storeError(error);
        float prevError = prevErrors[1]; //current error is stored in prevErrors[0]
        //Calculate the PID terms
        float proportionalTerm = Kp * error;
        float derivativeTerm =  Kd * (error - prevError);
        float integralTerm = Ki*getErrorSum();    
        output = proportionalTerm + derivativeTerm + integralTerm;
        return output;
    }

};
