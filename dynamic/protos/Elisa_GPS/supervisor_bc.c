#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64

#define ELISA_RADIS 0.015
float elisa_positon[2]; //存储elisa的xy坐标
float elisa_obstacles[4][2];//用一个正方形将elisa圈住，边长是2*ELISA_RADIS

void changeElisaObstacles(int x,int y){
  elisa_positon[0]=x,elisa_positon[1]=y;
  elisa_obstacles[0][0]=elisa_positon[0]-ELISA_RADIS,elisa_obstacles[0][1]=elisa_positon[1]-ELISA_RADIS;
  elisa_obstacles[1][0]=elisa_positon[0]+ELISA_RADIS,elisa_obstacles[1][1]=elisa_positon[1]-ELISA_RADIS;
  elisa_obstacles[2][0]=elisa_positon[0]+ELISA_RADIS,elisa_obstacles[2][1]=elisa_positon[1]+ELISA_RADIS;
  elisa_obstacles[3][0]=elisa_positon[0]-ELISA_RADIS,elisa_obstacles[0][1]=elisa_positon[1]+ELISA_RADIS;
}

int main() {
  wb_robot_init();

  // do this once only
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("elisa");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  while (wb_robot_step(32) != -1) {
    // this is done repeatedly
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("obstacle_elisa is at position: %g %g %g\n", trans[0], trans[1], trans[2]);
  }

  wb_robot_cleanup();

  return 0;
}