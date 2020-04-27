package my_code;

import control.EstimatorInterface;

import java.util.ArrayList;
import java.util.Random;
import java.lang.Math;

public class Localizer implements EstimatorInterface {
    private int rows;
    private int cols;
    private int head;
    private int trueR, trueC, trueH;
    private int readingR, readingC;
    private Random rand;
    private float[][] t_model;
    private float[][] t_model_T;
    private float[][][] s_model;
    private float[][] state;
    private float[] alphas;
    public ArrayList<Double> dists;
    public ArrayList<Double> guesses;
    public ArrayList<Double> sensor_readings;
    public ArrayList<Double> averages;
    public ArrayList<Double> averages_guesses;
    public ArrayList<Double> averages_sensor;
    private int stepCounter;

    public Localizer(int rows, int cols, int head) {
        this.rows = rows;
        this.cols = cols;
        this.head = head;

        rand = new Random();
        this.trueR = rand.nextInt(rows);
        this.trueC = rand.nextInt(cols);
        this.trueH = rand.nextInt(head);
        this.t_model = new float[rows*cols*head][rows*cols*head];
        this.s_model = new float[rows*cols+1][rows*cols*head][rows*cols*head];
        this.alphas = new float[rows*cols+1];
        this.state = new float[rows*cols*head][1];
        setUpTransitionModel();
        this.t_model_T = transposeMatrix(t_model);
        setUpSensorModel();
        setUpInitialState();

        this.dists = new ArrayList();
        this.averages = new ArrayList();
        this.guesses = new ArrayList<>();
        this.averages_guesses = new ArrayList<>();
        this.sensor_readings = new ArrayList<>();
        this.averages_sensor = new ArrayList<>();
        this.stepCounter = 0;
    }

    private void setUpTransitionModel(){
        for(int i = 0;i<rows; i++){
            for(int j = 0; j<cols; j++){
                for(int k = 0; k<head; k++){
                    switch (k){
                        case 0: //up
                            if (i == 0 && j == 0){ //top left
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.5f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.5f; //down
                            }else if(i== 0 && j == cols-1){ //top right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.5f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.5f; //left
                            }else if(i==rows-1 && j == 0){ //bottom left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.7f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.3f; //right
                            }else if(i==rows-1 && j == cols-1){ //bottom right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.7f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.3f; //left
                            }else if(i == 0){ //top
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 1f/3f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 1f/3f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 1f/3f; //left
                            }else if(i == rows-1){ //bot
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.7f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.15f; //right
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.15f; //left
                            }else if(j == 0){//left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.7f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.15f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.15f; //down
                            }else if(j == cols-1){//right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.7f; //up
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.15f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.15f; //left
                            }else{//center case
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.7f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.1f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.1f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.1f; //left
                            }
                            break;
                        case 1: //right
                            if (i == 0 && j == 0){ //top left
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.7f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.3f; //down
                            }else if(i== 0 && j == cols-1){ //top right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.5f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.5f; //left
                            }else if(i==rows-1 && j == 0){ //bottom left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.3f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.7f; //right
                            }else if(i==rows-1 && j == cols-1){ //bottom right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.5f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.5f; //left
                            }else if(i == 0){ //top
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.7f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.15f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.15f; //left
                            }else if(i == rows-1){ //bot
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.15f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.7f; //right
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.15f; //left
                            }else if(j == 0){//left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.15f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.7f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.15f; //down
                            }else if(j == cols-1){//right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 1/3f; //up
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 1/3f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 1/3f; //left
                            }else{//center case
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.1f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.7f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.1f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.1f; //left
                            }
                            break;
                        case 2: //down
                            if (i == 0 && j == 0){ //top left
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.3f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.7f; //down
                            }else if(i== 0 && j == cols-1){ //top right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.7f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.3f; //left
                            }else if(i==rows-1 && j == 0){ //bottom left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.5f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.5f; //right
                            }else if(i==rows-1 && j == cols-1){ //bottom right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.5f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.5f; //left
                            }else if(i == 0){ //top
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.15f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.7f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.15f; //left
                            }else if(i == rows-1){ //bot
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 1/3f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 1/3f; //right
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 1/3f; //left
                            }else if(j == 0){//left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.15f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.15f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.7f; //down
                            }else if(j == cols-1){//right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.15f; //up
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.7f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.15f; //left
                            }else{//center case
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.1f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.1f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.7f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.1f; //left
                            }
                            break;
                        case 3: //left
                            if (i == 0 && j == 0){ //top left
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.5f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.5f; //down
                            }else if(i== 0 && j == cols-1){ //top right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.3f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.7f; //left
                            }else if(i==rows-1 && j == 0){ //bottom left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.5f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.5f; //right
                            }else if(i==rows-1 && j == cols-1){ //bottom right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.3f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.7f; //left
                            }else if(i == 0){ //top
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.15f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.15f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.7f; //left
                            }else if(i == rows-1){ //bot
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.15f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.15f; //right
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.7f; //left
                            }else if(j == 0){//left
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 1/3f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 1/3f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 1/3f; //down
                            }else if(j == cols-1){//right
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.15f; //up
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.15f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.7f; //left
                            }else{//center case
                                this.t_model[i*cols*head+j*head+k][(i-1)*cols*head+j*head] = 0.1f; //up
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j+1)*head+1] = 0.1f; //right
                                this.t_model[i*cols*head+j*head+k][(i+1)*cols*head+j*head+2] = 0.1f; //down
                                this.t_model[i*cols*head+j*head+k][i*cols*head+(j-1)*head+3] = 0.7f; //left
                            }
                            break;
                    }
                }
            }
        }
    }

    private void setUpSensorModel(){
        for(int i = 0; i<rows;i++){
            for(int j = 0; j<cols; j++){
                int r,c;
                float sum = 0;
                for(int x = -2; x<3; x++){
                    r = i+x;
                    if(r<0 || r>=rows)
                        continue;
                    for(int y = -2; y<3; y++){
                        c = j+y;
                        if(c<0 || c>=cols)
                            continue;
                        float p = 0.1f;
                        if(Math.abs(x)==2 || Math.abs(y)==2){
                            p=1/40f;
                        }else if(Math.abs(x)==1 || Math.abs(y) == 1){
                            p=1/20f;
                        }
                        for(int k = 0; k<4;k++){
                            this.s_model[i*cols+j][r*cols*head+c*head+k][r*cols*head+c*head+k]=p;
                            sum+=p;
                        }
                    }
                }
                alphas[i*cols+j] = 1/sum;
            }
        }

        float sum = 0;
        for(int r = 0; r<rows;r++){
            for(int c = 0; c<cols; c++){
                float p = 1.0f;
                for(int i = -2; i<3; i++){
                    for(int j = -2; j<3; j++){
                        int x = r+i;
                        int y = c+j;
                        if ((Math.abs(i) == 2 || Math.abs(j) == 2) && x>=0 && x<rows && y>=0 && y<cols){
                            p-=0.025f;
                        }else if ((Math.abs(i) == 1 || Math.abs(j) == 1) && x>=0 && x<rows && y>=0 && y<cols) {
                            p -= 0.05f;
                        }else if ((Math.abs(i) == 0 || Math.abs(j) == 0) && x>=0 && x<rows && y>=0 && y<cols){
                            p-=0.1f;
                        }
                    }
                }
                for(int h = 0; h<head; h++){
                    this.s_model[rows*cols][r*cols*head+c*head+h][r*cols*head+c*head+h] = p;
                    sum+=p;
                }
            }
        }
        alphas[rows*cols] = 1/sum;

    }

    private void setUpInitialState(){
        float t = 1f/(rows*cols*head);
        for(int i = 0; i<rows*cols*head; i++){
            this.state[i][0] = t;
        }
    }

    public int getNumRows() {
        return this.rows;
    }

    public int getNumCols() {
        return this.cols;
    }

    public int getNumHead() {
        return this.head;
    }

    public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
        return this.t_model[x*cols*head+y*head+h][nX*cols*head+nY*head+nH];
    }

    /*
     * returns the probability entry of the sensor matrices O to get reading r corresponding
     * to position (rX, rY) when actually in position (x, y) (note that you have to take
     * care of potentially necessary transformations from states i = <x, y, h> to
     * positions (x, y)).
     */
    public double getOrXY(int rX, int rY, int x, int y, int h) {
        //System.out.println("Getting" + rX + " " + rY + " " + x + " "  + y + " " + h);
        if(rX == -1 && rY == -1)
            return (double)this.s_model[rows*cols][x*cols*head+y*head+h][x*cols*head+y*head+h];
        return (double)this.s_model[rX*cols+rY][x*cols*head+y*head+h][x*cols*head+y*head+h];
    }

    public int[] getCurrentTrueState() {
        int[] ret = new int[]{this.trueR, this.trueC, this.trueH};
        return ret;
    }

    public void updateCurrentReading(){
        ArrayList<int[]> list = new ArrayList<>();
        for(int i = 0; i<4; i++)
            list.add(new int[]{trueR,trueC});
        int r,c;
        for(int i = -2; i<3; i++){
            for(int j = -2; j<3; j++){
                r = trueR+i;
                c = trueC+j;
                if(r<0 || r>=rows || c<0 || c>= cols)
                    continue;
                if(Math.abs(i)==2 || Math.abs(j)==2){
                    list.add(new int[]{r,c});
                }else if(Math.abs(i)==1 || Math.abs(j) == 1){
                    list.add(new int[]{r,c});
                    list.add(new int[]{r,c});
                }
            }
        }
        int temp = rand.nextInt(40);
        if(temp<list.size()){
            int[] read =  list.get(temp);
            readingR = read[0];
            readingC = read[1];
        }else{
            readingR = -1;
            readingC = -1;
        }
    }

    public int[] getCurrentReading() {
        return new int[]{readingR,readingC};
    }

    public double getCurrentProb(int x, int y) {
        double ret = 0;
        for(int h = 0; h<head; h++){
            ret+=this.state[y*cols*head+x*head+h][0];
        }
        return ret;
    }

    public int[] getEstimate(){
        int rr = 0;
        int cc = 0;
        float top = 0f;
        for(int r = 0; r<rows; r++){
            for(int c = 0; c<cols; c++){
                for(int h = 0; h<head; h++){
                    if(state[r*cols*head+c*head+h][0] > top){
                        rr = r;
                        cc = c;
                        top = state[r*cols*head+c*head+h][0];
                    }
                }
            }
        }
        return new int[]{cc,rr};
    }

    public double manhattan(int[] a, int[] b){
        return Math.abs(a[0]-b[0])+Math.abs(a[1]-b[1]);
    }

    public int[] guessLocation(){
        return new int[]{rand.nextInt(rows),rand.nextInt(cols)};
    }

    public void moveRobot(){
        float temp = rand.nextFloat();
        int newR = 0;
        int newC = 0;
        int newH = 0;
        for(int r = 0; r<rows;r++){
            for(int c = 0; c<cols;c++){
                for(int h = 0; h<head; h++){
                    newR = r;
                    newC = c;
                    newH = h;
                    temp-=t_model[trueR*cols*head+trueC*head+trueH][r*cols*head+c*head+h];
                    if(temp<=0)
                        break;
                }
                if(temp<=0)
                    break;
            }
            if(temp<=0)
                break;
        }
        trueR = newR;
        trueC = newC;
        trueH = newH;
    }

    public void update() {
        moveRobot();
        updateCurrentReading();
        int[] reading = getCurrentReading();
        System.out.println("Reading " + reading[0] + " " + reading[1]);

        float[][] O;
        if(reading[0]==-1 && reading[1]==-1){
            O = s_model[rows*cols];
        }else{
            O = s_model[reading[1]*cols+reading[0]];
        }

        state = mul(t_model_T,state);
        state = mul(O,state);
        normalize(state);
        getCurrentTrueState();
        getEstimate();

        dists.add(manhattan(getCurrentTrueState(),getEstimate()));
        guesses.add(manhattan(getCurrentTrueState(),guessLocation()));
        if(reading[0]!=-1 && reading[1]!=-1){
            sensor_readings.add(manhattan(getCurrentTrueState(),reading));
        }
        stepCounter++;

        //normal
        String text = "Step " + stepCounter+":";
        double sum = 0;
        for(double d: dists){
            text += " " + d;
            sum+= d;
        }
        System.out.println(text);
        averages.add(sum/stepCounter);
        text = "Average:";
        for(double d: averages){
            text += " " + d;
        }
        System.out.println(text);

        //guesses
        text = "Guess:";
        sum = 0;
        for(double d: guesses){
            text += " " + d;
            sum+= d;
        }
        System.out.println(text);
        averages_guesses.add(sum/stepCounter);
        text = "Average:";
        for(double d: averages_guesses){
            text += " " + d;
        }
        System.out.println(text);

        //sensor
        text = "Sensor:";
        sum = 0;
        for(double d: sensor_readings){
            text += " " + d;
            sum+= d;
        }
        System.out.println(text);
        if(sensor_readings.size() > 0){
            averages_sensor.add(sum/sensor_readings.size());
        }else{
            averages_sensor.add(0.0);
        }
        text = "Average:";
        for(double d: averages_sensor){
            text += " " + d;
        }
        System.out.println(text);
    }

    public static float[][] mul(float[][] firstMatrix, float[][] secondMatrix) {
        int r1 = firstMatrix.length;
        int c1 = firstMatrix[0].length;
        int c2 = secondMatrix[0].length;
        float[][] product = new float[r1][c2];
        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
        return product;
    }

    public void normalize(float[][] m) {
        float sum = 0;
        for (int i = 0; i < m.length; i++)
            for (int j = 0; j < m[0].length; j++)
                sum+= m[i][j];
        float alpha = 1/sum;
        for (int i = 0; i < m.length; i++)
            for (int j = 0; j < m[0].length; j++)
                m[i][j] = m[i][j]*alpha;
    }

    public static float[][] transposeMatrix(float [][] m){
        float[][] temp = new float[m[0].length][m.length];
        for (int i = 0; i < m.length; i++)
            for (int j = 0; j < m[0].length; j++)
                temp[j][i] = m[i][j];
        return temp;
    }

    public static void displayMatrix(float[][] product) {
        float sum = 0;
        for(float[] row : product) {
            for (float column : row) {
                sum+=column;
                System.out.print(column + "    ");
            }
            System.out.println();
        }
        System.out.println("The sum is: "+sum);
    }
}
