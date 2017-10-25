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

/*********************************************************************************************************/
/* Averages the range and bearing sensor readings and outputs the average angle in radians.
/*    The average angle is the angle towards the center of the swarm relative to the sensors orientation.
/*********************************************************************************************************/
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
/*********************************************************************************************/
/*********************************************************************************************/
void CEPuckbrownian::ControlStep() 
{
   /* Get the highest reading in front of the robot, which corresponds to the closest object */
   Real fMaxReadVal  = m_pcProximity->GetReadings()[1];
   UInt32 unMaxReadIdx = 0;
  
       argos::LOG << "Distance to object = " << fMaxReadVal  << "'" << std::endl;
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
      
       SetWheelSpeedsFromVector(GetSwarmVelocity()); // Starts the flocking process
       //m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity);
       
   }

}

/*************************************************************************************************************/
/*  Returns a vector that represents the required velocity to reach the swarms center from an inital position*/
/************************************************************************************************************/
CVector2 CEPuckbrownian::GetSwarmVelocity()
{
  
  float x = m_pcPosSens->GetReading().Position.GetX();
  float y = m_pcPosSens->GetReading().Position.GetY();


  CVector2 swarmPosition;
  CVector2 currentPosition;
  CVector2 desiredPosition;
  CVector2 current_WheelVelocity;

  swarmPosition = CVector2(cos(getRadiansToSwarmCenter(m_pcRABSens)),sin(getRadiansToSwarmCenter(m_pcRABSens))); 
  currentPosition = CVector2(x,y);
  current_WheelVelocity = CVector2(m_fWheelVelocity,m_fWheelVelocity);

  swarmPosition -= currentPosition; /* postion of swarms center relative to a epucks positions i.e gives us our "desired position" */
  
  swarmPosition -= current_WheelVelocity; /* velocity needed to reach swarm */


  return swarmPosition;
   
}

/*****************************************************************************************************************/
/* Adjust the wheel speed of the epucks so it not only turns to the direction of the swarms center but           */
/* it also accelerates at a speed precise speed so the epucks will keep a good distancew(short range reuplsion)  */
/*****************************************************************************************************************/
void CEPuckbrownian::SetWheelSpeedsFromVector(const CVector2& c_heading) 
{
  float threshold = 2.5; //controls overall swarm density, article  sets to 2.5

  float number_of_robots = 10;//robots in the simulation

  float coherence = number_of_robots * (obstacleAvoidance_timer/1000); // the 'w' variable from the w-algorithm
  
  float x_speed = c_heading.GetX();
  float y_speed = c_heading.GetY();

   
  if(coherence > threshold)
   {
     m_pcWheels->SetLinearVelocity(x_speed ,y_speed);
   }

}
/*********************************************************************************************/
/*********************************************************************************************/
REGISTER_CONTROLLER(CEPuckbrownian, "epuck_brownian_controller")
