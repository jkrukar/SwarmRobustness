/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller for obstacle avoidance with the e-puck.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/epuck_brownian.argos
 */

#ifndef EPUCK_brownian_H
#define EPUCK_brownian_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/*Position Sensor*/
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the eye-bot light sensor */
#include <argos3/plugins/robots/eye-bot/control_interface/ci_eyebot_light_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>



using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckbrownian : public CCI_Controller {

public:

  struct SWheelTurningParams {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };


   /* Class constructor. */
   CEPuckbrownian();

   /* Class destructor. */
   virtual ~CEPuckbrownian() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_brownian_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset(); 

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   /*
    * Called by loop function to track distance swarm traveled
    */
   CVector2 getPosition();
   
   /*
    * Called by loop function sets failure case 0=none, 1, 2, 3
    */
   void setFailureCase(UInt32 num) { failure_case = num; }
   UInt32 getFailureCase() { return failure_case; }
   void setFailureTime(UInt32 num) { failure_time = num; }
   UInt32 getFailureTime() { return failure_time; }
   bool isFailed() { return is_wheels_failed || is_lights_sensor_failed || is_range_bearing_failed || is_proximity_sensor_failed; }
   bool isSensorsFailed() { return is_lights_sensor_failed || is_range_bearing_failed || is_proximity_sensor_failed; }
   void generateFailure();
 
   /*
    * Called by loop function determine if simulation is over
    */
   static UInt32 num_robots_task_completed;
   static UInt32 getNumRobotsFinished() { return num_robots_task_completed; }
   static void resetNumRobotsFinished() { num_robots_task_completed = 0; }
   
   /*
    * Methods implementing fail cases
    */
   static std::vector<CVector2> failed_epuck_list;
   virtual void SetLinearVelocity(Real f_left_velocity, Real f_right_velocity);
   bool determineIfFailedEpuck(float f_angle, float f_range);

protected:

virtual CVector2 GetSwarmVelocity();
/*
    * Calculates the flocking interaction vector.
    */
  
   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
void SetWheelSpeedsFromVector(const CVector2& c_heading);

CVector2 getVectorToSwarm(CCI_RangeAndBearingSensor* sensor);

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the e-puck proximity sensor */
   CCI_ProximitySensor* m_pcProximity;
   /*Pointer to the e-puck positioning sensor*/
   CCI_PositioningSensor* m_pcPosSens;
   /*Pointer to the range and bearing sensor*/
   CCI_RangeAndBearingSensor* m_pcRABSens;
   /*Pointer to the light sensor*/
   CCI_EyeBotLightSensor* m_pcLightSens;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;

   // Timer for last object avoided
   float obstacleAvoidance_timer;
   int tickCounter = 0;
   int reachedGoal = 0;

   // Tracks beacon visibility state. If becaon is not visible 0, If beacon is visible 1.
   int beaconVisible = 0;

   // Variables implementing fail cases
   int failure_case = 0;
   int failure_time = 0;
   bool is_wheels_failed = false;
   bool is_lights_sensor_failed = false;
   bool is_range_bearing_failed = false;
   bool is_proximity_sensor_failed = false;
   
   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_brownian_controller> section.
    */
   /* Wheel speed. */
   Real m_fWheelVelocity;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;
};

#endif

