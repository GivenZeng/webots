#include <webots/robot.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <webots/receiver.h>
#include "../../lib/odometry.h"

#define PI 3.14159265358979
#define LEFT 0
#define RIGHT 1
#define VERBOSE 1
#define SHOW_SPEED 0
#define SHOW_POSITION 0
#define SHOW_ABSTACLE_POSITION 1
#define SHOW_DISTANCE_TO_GOAL 0
#define TIME_STEP 64
#define MAP_WIDTH 0.42
#define MAP_HEIGHT 0.30

// only one obstacle, vertices ordered clockwise
#define OBSTACLE_SIZE 3
float obstacle[OBSTACLE_SIZE][2] = {
  {0.1,0.1},
  {0.09,0.2},
  {0.2,0.1}
};

//球用四个点围起来
#define BALL_OBSTACLE_SIZE 4
#define BALL_RADIS 0.001
#define BALL_NUM 2
float balls[BALL_NUM][2]={
  {0.2,0.3},
  {0.4,0.3}
};
float ball_obstacles[BALL_NUM][BALL_OBSTACLE_SIZE][2]={
  {
    {0.2-BALL_RADIS,0.3-BALL_RADIS},
    {0.2+BALL_RADIS,0.3-BALL_RADIS},
    {0.2+BALL_RADIS,0.3+BALL_RADIS},
    {0.2-BALL_RADIS,0.3+BALL_RADIS}
  },
  {
    {0.4-BALL_RADIS,0.3-BALL_RADIS},
    {0.4+BALL_RADIS,0.3-BALL_RADIS},
    {0.4+BALL_RADIS,0.3+BALL_RADIS},
    {0.4-BALL_RADIS,0.3+BALL_RADIS}
  }
};

/*******************elisa data***********************/
#define ELISA_OBSTACLE_SIZE 4
#define ELISA_RADIS 0.02
float elisa_position[2]={0.24,0.12};
float elisa_obstacles[4][2];
// wheel
#define SPEED_UNIT   0.00628
#define ENCODER_UNIT 159.23
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor,receiver;

// the goal
float goal_x = 0.36;
float goal_y = 0.245;

// attractive and repulsive factors
float k_att = 15.0;
float k_rep = 10.0;

// maximum distance of influence for the obstacles
float rho_0 = 0.08;

// speed and previous speed of the robot
int speed[2]={0,0};
int pspeed[2]={0,0};

int finished = 0;

// size of the display
int width, height;

// Instantiate odometry track structure
struct sOdometryTrack ot;

WbDeviceTag display;
WbImageRef background;

static void init_display();
static void draw_on_display();
static void set_speed(int l, int r);
static float compute_force(float pos_x, float pos_y, float* force_x, float* force_y);
static float distance_to_segment(float cx, float cy, float ax, float ay,
                                 float bx, float by, float* obs_x, float* obs_y);
static float distance_to_obstacle(float x, float y, float* vec_x, float* vec_y);
static float compute_force_of_all_obstacle(float pos_x, float pos_y, float* force_x, float* force_y);
static float distance_to_ball_obstacle(float x, float y,int ballNum, float* vec_x, float* vec_y);
static int run(void);


/***************************my code******************************/
//在本处，z和y是等价的。因此有时候命名为y，有时为z。无论如何表示的是elisa的平面坐标
static void changeElisaObstacles(float x,float z);
static float distance_to_elisa(float x, float y, float* vec_x, float* vec_y);
static void getElisaPosition();

//获取elisa的位置
static void getElisaPosition(){
  // while(wb_receiver_get_queue_length(receiver) > 0){
  //   //读取平面坐标
  //     const float * data=wb_receiver_get_data(receiver);
  //     if(VERBOSE>0&&SHOW_ABSTACLE_POSITION>0)
  //       printf("epuck get elisa's position:%g,%g\n",data[0],data[1]);
  //     changeElisaObstacles(data[0],data[1]);
  //     wb_receiver_next_packet(receiver);
  // }

  while (wb_receiver_get_queue_length(receiver) > 0) 
    {
        //接收robot的位置及角度消息
        const double *a = wb_receiver_get_data(receiver);
        if(VERBOSE>0&&SHOW_ABSTACLE_POSITION>0)
          printf("epuck get elisa's position:%g,%g\n",a[0],a[1]);
        changeElisaObstacles(a[0],a[1]);
        wb_receiver_next_packet(receiver);
     }
}
//计算正方形的四个点，这里使用的是逆时针 todo：统一改为顺时针
static void changeElisaObstacles(float x,float z){
  printf("the elisa position is%g,%g\n",x,z);
  elisa_position[0]=x,elisa_position[1]=z;
  elisa_obstacles[0][0]=elisa_position[0]-ELISA_RADIS,elisa_obstacles[0][1]=elisa_position[1]-ELISA_RADIS;
  elisa_obstacles[1][0]=elisa_position[0]+ELISA_RADIS,elisa_obstacles[1][1]=elisa_position[1]-ELISA_RADIS;
  elisa_obstacles[2][0]=elisa_position[0]+ELISA_RADIS,elisa_obstacles[2][1]=elisa_position[1]+ELISA_RADIS;
  elisa_obstacles[3][0]=elisa_position[0]-ELISA_RADIS,elisa_obstacles[3][1]=elisa_position[1]+ELISA_RADIS;
}


/*
  计算点xy到elisa的距离，以及到达elisa的方向
*/
static float distance_to_elisa(float x, float y, float* vec_x, float* vec_y){
  float dist = -1;
  *vec_x = 0.0;
  *vec_y = 0.0;
  float cand_x = 0.0;
  float cand_y = 0.0;
  int i = 0;
  int inObstacle = 1;

  // for each segment of the obstacle
  for (i = 0; i < ELISA_OBSTACLE_SIZE; i++) {
    inObstacle = 1;

    int ip = i+1;
    if (ip > ELISA_OBSTACLE_SIZE) {
      ip = 1;
    }

    float d = distance_to_segment(x, y,
            elisa_obstacles[i][0], elisa_obstacles[i][1],
            elisa_obstacles[(i+1)%ELISA_OBSTACLE_SIZE][0], elisa_obstacles[(i+1)%ELISA_OBSTACLE_SIZE][1],
            &cand_x, &cand_y);

    if (d != -1) {
      inObstacle = 0;
    }

    if (d < dist || dist == -1) {
      dist = d;
      *vec_x = cand_x;
      *vec_y = cand_y;
    }
  }

  if (inObstacle == 1) {
    dist = -1;
    *vec_x = 0.0;
    *vec_y = 0.0;
  }

  return dist;
}
static void init_display(){
  int i,j,color,pot;
  float fx, fy;
  width = wb_display_get_width(display);
  height = wb_display_get_height(display);

  for (i = 0; i < width; i++) {
    for (j = 0; j < height; j++) {
      pot = (int)(70 * compute_force(MAP_WIDTH * i / width, MAP_HEIGHT * j / height, &fx, &fy));
      pot = abs(pot) > 255 ? 255 : pot;
      pot = pot < 0 ? 255 : pot;
      color = 0x00010101*(255 - pot);
      wb_display_set_color(display,color);
      wb_display_draw_rectangle(display,i,height-j-1,1,1);
    }
  }

  background = wb_display_image_copy(display,0,0,width,height);
  wb_display_set_color(display,0xFF0000);
}

static void draw_on_display()
{
  wb_display_image_paste(display,background,0,0, false);

  int i = (int)(width * ot.result.x / MAP_WIDTH);
  int j = (int)(height * ot.result.y / MAP_HEIGHT);

  wb_display_draw_rectangle(display,i,height-j-1,1,1);
}

/*
 * This functions sets the speed of both motors sending the command if and only if
 * the speed has changed.
 */
static void set_speed(int l, int r)
{
  pspeed[LEFT] = speed[LEFT];
  pspeed[RIGHT] = speed[RIGHT];
  speed[LEFT] = l;
  speed[RIGHT] = r;
  float left_speed=SPEED_UNIT * speed[LEFT];
  float right_speed=SPEED_UNIT * speed[RIGHT];
  while(true){
     if(left_speed>6.28||right_speed>6.28){
      left_speed=left_speed/1.1;
      right_speed=right_speed/1.1;
      if(VERBOSE>0&&SHOW_SPEED>0) printf("speed is %f,%f\n", left_speed,right_speed);
    }else{
      break;
    }
  }
  if (pspeed[LEFT] != speed[LEFT] || pspeed[RIGHT] != speed[RIGHT]) {
    wb_motor_set_velocity(left_motor, left_speed  );
    wb_motor_set_velocity(right_motor, right_speed);
  }

}


/*
 * This functions computes the force applied to the robot at point (pos_x,pos_y)
 * The result is copied in (force_x,force_y) we return the potential at the same point.
 */
static float compute_force(float pos_x, float pos_y, float* force_x, float* force_y)
{
  /*
   * TODO: Complete the function such that we can compute the potenatial
   * and the applied force on the robot.
   */

  float vec_x, vec_y, tx, ty;
  float f_x = 0.0;
  float f_y = 0.0;
  float pot = 0.0;
  float d = distance_to_obstacle(pos_x, pos_y, &vec_x, &vec_y);
  float d_goal = sqrt((goal_x-pos_x)*(goal_x-pos_x) + (goal_y-pos_y)*(goal_y-pos_y));

  // attractive part (go to the goal)
  f_x += k_att * (goal_x - pos_x) / d_goal;
  f_y += k_att * (goal_y - pos_y) / d_goal;

  pot += k_att * d_goal * d_goal;

  // repulsive part (avoid obstacles)
  if (d < rho_0 && d != 0 && d != -1) {
    tx = k_rep * (1/d - 1/rho_0) / d*d * (-vec_x);
    ty = k_rep * (1/d - 1/rho_0) / d*d * (-vec_y);
    f_x += tx;
    f_y += ty;

    pot += k_rep * (1/d - 1/rho_0) * (1/d - 1/rho_0);
  }

  *force_x = f_x;
  *force_y = f_y;

  return pot;
}
static float compute_force_of_all_obstacle(float pos_x, float pos_y, float* force_x, float* force_y)
{
  /*
   * TODO: Complete the function such that we can compute the potenatial
   * and the applied force on the robot.
   */

  float vec_x, vec_y, tx, ty;
  float f_x = 0.0;
  float f_y = 0.0;
  float pot = 0.0;
  float d_goal = sqrt((goal_x-pos_x)*(goal_x-pos_x) + (goal_y-pos_y)*(goal_y-pos_y));
  // attractive part (go to the goal)
  f_x += k_att * (goal_x - pos_x) / d_goal;
  f_y += k_att * (goal_y - pos_y) / d_goal;
  pot += k_att * d_goal * d_goal;

  // repulsive part (avoid obstacles)
  float d = distance_to_obstacle(pos_x, pos_y, &vec_x, &vec_y);
  if (d < rho_0 && d != 0 && d != -1) {
    tx = k_rep * (1/d - 1/rho_0) / d*d * (-vec_x);
    ty = k_rep * (1/d - 1/rho_0) / d*d * (-vec_y);
    f_x += tx;
    f_y += ty;

    pot += k_rep * (1/d - 1/rho_0) * (1/d - 1/rho_0);
  }
  //计算epuck到球的距离，计算力的影响
  for(int i=0;i<BALL_NUM;i++){
    float d_to_ball=distance_to_ball_obstacle(pos_x,pos_y,i,&vec_x,&vec_y);
    if(d_to_ball<rho_0&&d_to_ball!=0&&d!=-1){
        tx = k_rep * (1/d - 1/rho_0) / d*d * (-vec_x);
        ty = k_rep * (1/d - 1/rho_0) / d*d * (-vec_y);
        f_x += tx;
        f_y += ty;

        pot += k_rep * (1/d - 1/rho_0) * (1/d - 1/rho_0);
    }
  }
  //计算epuck到elisa的距离，并计算力的影响
  d=distance_to_elisa(pos_x,pos_y,&vec_x,&vec_y);
  if (d < rho_0 && d != 0 && d != -1) {
    tx = k_rep * (1/d - 1/rho_0) / d*d * (-vec_x);
    ty = k_rep * (1/d - 1/rho_0) / d*d * (-vec_y);
    f_x += tx;
    f_y += ty;

    pot += k_rep * (1/d - 1/rho_0) * (1/d - 1/rho_0);
  }

  *force_x = f_x;
  *force_y = f_y;

  return pot;
}


/*
 * Compute the distance from the point (x,y) to the segment
 * determined by the points (ax,ay) and (bx,by). Returns -1 if
 * the point is not on the clockwise side of the segment. Otherwise,
 * (vec_x,vec_y) contains a unit vector representing the direction of
 * the closest point of segment.
 *
 * stores the obstacle point considered in (obs_x, obs_y) and returns the distance
 */
static float distance_to_segment(float x, float y, float ax, float ay ,
                                 float bx, float by, float* vec_x, float* vec_y)
{
  // lets compute useful vectors
  float v12_x = bx - ax; // vector from 1st segment point to 2nd point
  float v12_y = by - ay;
  float v1p_x = x - ax; // vector from 1st segment point to test point
  float v1p_y = y - ay;
  float norm_v12 = sqrt(v12_x*v12_x + v12_y*v12_y);
  float norm_v1p = sqrt(v1p_x*v1p_x + v1p_y*v1p_y);
  float dot_v12_v1p = v12_x*v1p_x + v12_y*v1p_y;

  float vx, vy, dist;
/*
  // We can determine using the third composant of the cross product
  // if the point is on the wrong side.
  float tmp = v12_x*v1p_y - v12_y*v1p_x;
  if (tmp < 0) {
    *vec_x = 0;
    *vec_y = 0;
    return -1;
  }
*/
  // Now, we project v1p on  v12 and get the norm.
  float n = dot_v12_v1p / norm_v12;
  if (n <= 0) {
    // the closest point is the segment start
    dist = sqrt((x-ax)*(x-ax) + (y-ay)*(y-ay));//norm([x y]-seg(1, :));
    vx = ax - x;
    vy = ay - y;//seg(1, :) - [x y];
  } else if (n >= norm_v12) {
    // the closest point is the segment end
    dist = sqrt((x-bx)*(x-bx) + (y-by)*(y-by));//norm([x y]-seg(2, :));
    vx = bx - x;
    vy = by - y;//seg(2, :) - [x y];
  } else {
    // we need to project to get the closest distance
    dist = norm_v1p * sin(acos(dot_v12_v1p/norm_v12/norm_v1p));
    vx = v12_x * dot_v12_v1p / (norm_v12*norm_v12) - v1p_x;
    vy = v12_y * dot_v12_v1p / (norm_v12*norm_v12) - v1p_y;//v12 * dot_v12_v1p / (norm_v12*norm_v12) - v1p;
  }

  float norm_v = sqrt(vx*vx + vy*vy);
  if (norm_v > 0) {
    vx = vx / norm_v;
    vy = vy / norm_v;//vec/norm(vec);
  }

  *vec_x = vx;
  *vec_y = vy;

  return dist;
}


/*
 * Compute the shortest distance from the position (x,y) to the obstacle.
 * If dist is not -1, then (vec_x, vec_y) contains the normal vector
 * pointing from (x,y) to the closest obstacle.
 */
static float distance_to_obstacle(float x, float y, float* vec_x, float* vec_y)
{
  float dist = -1;
  *vec_x = 0.0;
  *vec_y = 0.0;
  float cand_x = 0.0;
  float cand_y = 0.0;
  int i = 0;
  int inObstacle = 1;

  // for each segment of the obstacle
  for (i = 0; i < OBSTACLE_SIZE; i++) {
    inObstacle = 1;

    int ip = i+1;
    if (ip > OBSTACLE_SIZE) {
      ip = 1;
    }

    float d = distance_to_segment(x, y,
            obstacle[i][0], obstacle[i][1],
            obstacle[(i+1)%OBSTACLE_SIZE][0], obstacle[(i+1)%OBSTACLE_SIZE][1],
            &cand_x, &cand_y);

    if (d != -1) {
      inObstacle = 0;
    }

    if (d < dist || dist == -1) {
      dist = d;
      *vec_x = cand_x;
      *vec_y = cand_y;
    }
  }

  if (inObstacle == 1) {
    dist = -1;
    *vec_x = 0.0;
    *vec_y = 0.0;
  }

  return dist;
}

//计算x，y到球的距离以及到球的防线
static float distance_to_ball_obstacle(float x, float y,int ballNum, float* vec_x, float* vec_y)
{
  float dist = -1;
  *vec_x = 0.0;
  *vec_y = 0.0;
  float cand_x = 0.0;
  float cand_y = 0.0;
  int i = 0;
  int inObstacle = 1;

  // for each segment of the ball_obstacle
  for (i = 0; i < BALL_OBSTACLE_SIZE; i++) {
    inObstacle = 1;

    int ip = i+1;
    if (ip > BALL_OBSTACLE_SIZE) {
      ip = 1;
    }

    float d = distance_to_segment(x, y,
            ball_obstacles[ballNum][i][0], ball_obstacles[ballNum][i][1],
            ball_obstacles[ballNum][(i+1)%BALL_OBSTACLE_SIZE][0], ball_obstacles[ballNum][(i+1)%BALL_OBSTACLE_SIZE][1],
            &cand_x, &cand_y);

    if (d != -1) {
      inObstacle = 0;
    }

    if (d < dist || dist == -1) {
      dist = d;
      *vec_x = cand_x;
      *vec_y = cand_y;
    }
  }

  if (inObstacle == 1) {
    dist = -1;
    *vec_x = 0.0;
    *vec_y = 0.0;
  }

  return dist;
}


/*
 * This function compute the difference of two angles
 */
static float angleDiff(float angle1, float angle2)
{
  // ensure both angle are withing [0, 2*PI]
  while (angle1 < 0) {
    angle1 += 2*PI;
  }
  while (angle1 > 2*PI) {
    angle1 -= 2*PI;
  }
  while (angle2 < 0) {
    angle2 += 2*PI;
  }
  while (angle2 > 2*PI) {
    angle2 -= 2*PI;
  }

  // compute difference
  float d = angle1 - angle2;

  while (d < -PI) {
    d += 2*PI;
  }
  while (d > PI) {
    d -= 2*PI;
  }

  return d;
}


/*
 * This is the main control loop function, it is called repeatedly by Webots
 */
static int run(void)
{
  float fx, fy;
  float x, y, theta;
  int speed_l, speed_r;

  odometry_track_step_pos(&ot,
                          ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                          ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));
  x = ot.result.x;
  y = ot.result.y;
  theta = ot.result.theta;

  draw_on_display();

  float d_goal = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));

  if (d_goal > 0.005) {
    compute_force_of_all_obstacle(x, y, &fx, &fy);
    if (VERBOSE > 0&&SHOW_POSITION) printf("Current position is (%f,%f,%f), current force is (%f,%f)\n",x,y,theta,fx,fy);
    if (VERBOSE > 0&&SHOW_DISTANCE_TO_GOAL) printf("Distance to goal is %f\n", d_goal);

    /*
     *  TODO: Write the robot control code.
     */

    float d_angle = angleDiff(atan2(fy, fx), theta);

    float ang_speed = 100 * d_angle;
    float base_speed = 15 * sqrt(fx*fx + fy*fy) * cos(d_angle);

    speed_l = base_speed - ang_speed;
    speed_r = base_speed + ang_speed;

    set_speed(speed_l, speed_r);
  } else {
    set_speed(0, 0);
    if (finished != 1) {
      if(VERBOSE) printf("Goal is reached.\n");
      finished++;
    }
  }

  return TIME_STEP; /* this is the time step value, in milliseconds. */
}

/*
 * This is the main program which sets up the reset and run function.
 */
int main()
{
  wb_robot_init(); /* initialize the webots controller library */
  changeElisaObstacles(elisa_position[0],elisa_position[1]);
  // get a handler to the motors and set target position to infinity (speed control).
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  receiver = wb_robot_get_device("receiver");
  wb_receiver_set_channel(receiver, 6666);
  wb_receiver_enable(receiver, TIME_STEP);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // get a handler to the position sensors and enable them.
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  // robot init speed is 0
  speed[LEFT]=speed[RIGHT]=0;
  pspeed[LEFT]=pspeed[RIGHT]=0;
  wb_motor_set_velocity(left_motor, SPEED_UNIT * speed[LEFT]);
  wb_motor_set_velocity(right_motor, SPEED_UNIT * speed[RIGHT]);

  // required to get the position sensor values
  wb_robot_step(TIME_STEP);

  // Initializes tracking structure
  odometry_track_start_pos(&ot,
                          ENCODER_UNIT * wb_position_sensor_get_value(left_position_sensor),
                          ENCODER_UNIT * wb_position_sensor_get_value(right_position_sensor));

  // start position
  ot.result.x = 0.04;
  ot.result.y = 0.04;
  ot.result.theta = 0;

  // get display device
  display = wb_robot_get_device("display");
  init_display();

  printf("Reset OK\n");

  /* main loop */
  while(wb_robot_step(TIME_STEP) != -1) {
    getElisaPosition();
    // while(wb_receiver_get_queue_length(receiver) > 0){
    // //读取平面坐标
    // const float * data=wb_receiver_get_data(receiver);
    // if(VERBOSE>0&&SHOW_ABSTACLE_POSITION>0)
    //   printf("epuck get elisa's position:%g,%g\n",data[0],data[1]);
    // changeElisaObstacles(data[0],data[1]);
    // wb_receiver_next_packet(receiver);
    // }
    run();
  }

  wb_robot_cleanup();

  return 0;
}
