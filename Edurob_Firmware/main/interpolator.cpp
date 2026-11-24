
#include "simpleinterpol/interpolator.h"

// include parameters file
#include "include/parameters.h"
using namespace imsl;
using namespace imsl::interpolator;

bool interpolator_inited = false;

std::vector<RotAndTrans<double> *> segments;
int current_segment = 0;   // current segment accessed by the interpolator
double current_time = 0.0; // current time in ms

void init_interpol(double trans_vel, double trans_accel, double rot_vel, double rot_accel, PoseV<double> points[], int num_points)
{
  for (int i = 0; i < num_points - 1; i++) {
    segments.push_back(new RotAndTrans<double>(points[i], points[i + 1], trans_vel, trans_accel, rot_vel, rot_accel));
    printf("Segment %d: created with Point1: X:%f, Y:%f, Theta: %f, Transvel: %f, TransAccel: %f, RotVel: %f, RotAccel: %f\n", i, points[i].x, points[i].y, (double) points[i].theta, trans_vel, trans_accel, rot_vel, rot_accel);
    printf("Segment %d: created with Point2: X:%f, Y:%f, Theta: %f\n", i, points[i + 1].x, points[i + 1].y, (double) points[i + 1].theta);
  }
  xSemaphoreTake(xInterpolMutex, portMAX_DELAY);
  interpolator_inited = true;
  xSemaphoreGive(xInterpolMutex);
}

PoseV<double> interpol_itteration()
{
  // check if the current segment is finished
  if (current_time + sampleTime / 1000.0 > segments[current_segment]->getTg()) {
    if (current_segment + 1 >= segments.size()) // check if the last segment is reached and return a pose with -1 if so
    {
      PoseV<double> empty_pose;
      empty_pose.x = -1.0;
      empty_pose.y = -1.0;
      empty_pose.theta = -1.0;
      empty_pose.vx = -1.0;
      empty_pose.vy = -1.0;
      empty_pose.omega = -1.0;
      return empty_pose;
    } else { // if not, go to the next segment and reset the time
      current_segment++;
      current_time = 0.0;
      return segments[current_segment]->Interpolation(current_time);
    }
  } else {
    // Return the Interpolated Pose with the current time
    current_time += sampleTime / 1000.0;
    return segments[current_segment]->Interpolation(current_time);
  }
}

void interpol_task(void *args)
{
  while (true) {
    PathplanningData *pathplanningStatus = (PathplanningData *) args;

    // Dummy Variables to be able to access the Mutex
    bool intpol_state_safer = NULL;
    bool points_send_state_safer = NULL;

    // Check if the Interpol is already inited
    xSemaphoreTake(xInterpolMutex, portMAX_DELAY);
    intpol_state_safer = interpolator_inited;
    xSemaphoreGive(xInterpolMutex);

    // If the Interpol is not inited, wait for the Points to be send from the Webserver
    if (intpol_state_safer == false) {

      while (true) { // Wait for Points to be send from the Webserver

        // Check if the Points are send
        xSemaphoreTake(xPointsMutex, portMAX_DELAY);
        points_send_state_safer = pointsSend;
        xSemaphoreGive(xPointsMutex);

        if (points_send_state_safer) {
          xSemaphoreTake(xPointsMutex, portMAX_DELAY);
          pointsSend = false; // Reset the PointsSend so that the Interpolation can be started again
          pathplanningStatus->points.clear();
          xSemaphoreGive(xPointsMutex);
          break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Check every 1s if the Points are send
      }

      // Init the Interpol with the Points from the Webserver
      PoseV<double> *points = &pathplanningStatus->points[0]; // Convert vector to array
      init_interpol(
        pathplanningStatus->trans_vel,
        pathplanningStatus->trans_accel,
        pathplanningStatus->rot_vel,
        pathplanningStatus->rot_accel,
        points, // Conversion from Degree to Rad happened in the Webserver
        pathplanningStatus->num_points);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // Check if the Interpol is over every 500ms
  }
}
