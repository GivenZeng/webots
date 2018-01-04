
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>
#include <webots/emitter.h>

#define TIME_STEP 64

#define VERBOSE 0
WbDeviceTag emitter;

//用于将elisa的坐标发给epuck
int main() {
  wb_robot_init();

  // do this once only
  emitter=wb_robot_get_device("emitter");
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("elisa");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  while (wb_robot_step(TIME_STEP) != -1) {
    //获取elisa的坐标
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    if(VERBOSE>0) printf("elisa is at position: %g %g %g\n", trans[0], trans[1], trans[2]);
    double elisaPosition[2]={trans[0],trans[2]};
    wb_emitter_send(emitter, elisaPosition, 2 * sizeof(double));
  }

  wb_robot_cleanup();

  return 0;
}