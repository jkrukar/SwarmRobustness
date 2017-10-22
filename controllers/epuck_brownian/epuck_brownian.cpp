/* Include the controller definition */
#include "epuck_brownian.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <cmath>
#include<bits/stdc++.h>

/****************************************/
/****************************************/

CEPuckbrownian::CEPuckbrownian() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABSens(NULL),
   m_pcLightSens(NULL),
   m_pcPosSens(NULL),
   m_fWheelVelocity(2.5f) {}
   

/****************************************/
/****************************************/

void CEPuckbrownian::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_brownian><actuators> and
    * <controllers><epuck_brownian><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
   m_pcPosSens = GetSensor  <CCI_PositioningSensor             >("positioning"    );
   m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor    >("range_and_bearing" );
   m_pcLightSens = GetSensor  <CCI_EyeBotLightSensor        >("eyebot_light"      );


   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/* Averages the range and bearing sensor readings and outputs the average angle in radians.
    The average angle is the angle towards the center of the swarm relative to the sensors orientation.*/
/****************************************/
float getRadiansToSwarmCenter(CCI_RangeAndBearingSensor* m_pcRABSens){

  float averageRadians = 0;

  //Get average of range and bearing sensor readings
  argos::CCI_RangeAndBearingSensor::TReadings rabReadings = m_pcRABSens->GetReadings();
  int rabReadingCount = rabReadings.size();
  float nextReading = 0;
  float sinMean = 0;
  float cosMean = 0;

  for(int i=0;i<rabReadingCount;i++){

    nextReading = rabReadings[i].HorizontalBearing.UnsignedNormalize().GetValue();

    /*Add sin and cos values of the angle to the sin and cos total.*/
    sinMean += sin(nextReading);
    cosMean += cos(nextReading);
  }

  /*Divide the sin and cos total by the number of readings to get the average*/
  sinMean /= rabReadingCount;
  cosMean /= rabReadingCount;

  /*Convert to degrees for atan()*/
  sinMean = sinMean*M_PI/180;
  cosMean = cosMean*M_PI/180;

  /*Take the arctangent to get the mean angle*/
  averageRadians = atan(sinMean/cosMean);

  if(cosMean < 0){
    averageRadians += M_PI;
  }else if(sinMean < 0){
    averageRadians += M_PI*2;
  }

  /*TODO: Remove eventually. Left in for debugging/demonstration purposes*/
  float averageAngle = averageRadians*180/M_PI;
  
  //argos::LOG << "averageAngle = " << averageAngle << "'" << std::endl;

  return averageRadians;
}
//**********************************************************************************/
/*                  Flocking long-range attraction(steering/cohesion)              */
/*   Parameters@                                                                   */
/*   obstacle_avoidance_timer = time between each obstacle encounter               */
/*   m_pcRABSens = Check Documentation                                             */
/*   m_pcWheels controls the velocity of the epucks                                */
/*   current_X,current_y = current x and y position                                */
//**********************************************************************************/
void flocking_long_range(float obstacle_avoidance_timer,CCI_RangeAndBearingSensor* m_pcRABSens,     
              CCI_DifferentialSteeringActuator* m_pcWheels,float current_x,float current_y)
{

  
  float threshold = 2.5; //controls overall swarm density, article says to set to 2.5
  float number_of_robots = 2;//robots in the simulation
  float coherence = 0;// the 'w' variable from the w-algorithm

  float swarm_x = cos(getRadiansToSwarmCenter(m_pcRABSens)); // get the x coordinate of the swarm
  float swarm_y = sin(getRadiansToSwarmCenter(m_pcRABSens)); // get the  y coordinate of the swarm

  float desired_x_position = swarm_x - current_x;
  float desired_y_position = swarm_y - current_y;

  float right_wheelVelocity = desired_x_position - 2.0f;  // set to 2.0f for testing purposes, should be current velocity of the "lead" epuck in the swarm
  float left_Wheelveloctiy = desired_y_position - 2.0f;  //
  
  coherence = number_of_robots * obstacle_avoidance_timer;
              
   /* Turn the robot towards center of swarm  */
   if( coherence > threshold)
    {
       m_pcWheels->SetLinearVelocity(right_wheelVelocity, left_Wheelveloctiy);
    }
}

/*********************************************************************************************/
/*********************************************************************************************/
void CEPuckbrownian::ControlStep() {

    timerToTurn++;
   /* Get the highest reading in front of the robot, which corresponds to the closest object */
   Real fMaxReadVal = m_pcProximity->GetReadings()[0];
   UInt32 unMaxReadIdx = 0;
  
   
   if(fMaxReadVal < m_pcProximity->GetReadings()[1]) {
      fMaxReadVal = m_pcProximity->GetReadings()[1];
      unMaxReadIdx = 1;
   }
   if(fMaxReadVal < m_pcProximity->GetReadings()[7]) {
      fMaxReadVal = m_pcProximity->GetReadings()[7];
      unMaxReadIdx = 7;
   }
   if(fMaxReadVal < m_pcProximity->GetReadings()[6]) {
      fMaxReadVal = m_pcProximity->GetReadings()[6];
      unMaxReadIdx = 6;
   }
   /* Do we have an obstacle in front? */
   if(fMaxReadVal > 0.0f) {
     obstacleAvoidance_timer = 0; // timer stays 0 until we stop avoiding an object
     /* Yes, we do: avoid it */
     if(unMaxReadIdx == 0 || unMaxReadIdx == 1) {
       /* The obstacle is on the left, turn right */
       m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
       
     }
     else {
       /* The obstacle is on the left, turn right */
       m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
     }
   }

  
   else {
     obstacleAvoidance_timer++; // time since last obstacle avoidance
     /* Go towards goal */ 
     // m_pcWheels->SetLinearVelocity(3.0f, 3.0f);
      
   }
  
/***********************************************************************************************/
/* The code below is for testing purposes only, currently im working with two epucks trying    */
/*  to get "fb1" to follow "fbo" from the flocking functionabove.                             */ 
/**********************************************************************************************/ 
  float x= m_pcPosSens->GetReading().Position.GetX();
  float y= m_pcPosSens->GetReading().Position.GetY();

  std::string id = this->GetId(); //Get id so that we can limit the log output to individual robots
  std::string robots[2] = {id}; /* Number of epucks */  
       
                       
  //loop through each epuck
  for(int i = 0; i < 2; i++)
     {  
         // Make the first epuck head in some direction simulating if it were heading to a goal
         if(robots[i] == "fb0")
         {
            m_pcWheels->SetLinearVelocity(2.0f,2.0f);
            //argos::LOG << "averageAngle = " << getRadiansToSwarmCenter(m_pcRABSens) << "'" << std::endl;     
         }   
         // Call flocking function for the second epuck to make it follow
         if(robots[i] == "fb1")
         {
            flocking_long_range(obstacleAvoidance_timer,m_pcRABSens,m_pcWheels,x,y);        
            argos::LOG << "averageAngle = " << getRadiansToSwarmCenter(m_pcRABSens) << "'" << std::endl;     
         }   
     }

}
/***********************************************************************************************/
/**********************************************************************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckbrownian, "epuck_brownian_controller")
