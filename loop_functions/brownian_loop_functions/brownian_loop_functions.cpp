#include "brownian_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/epuck_brownian/epuck_brownian.h>

CBrownianLoopFunctions::CBrownianLoopFunctions() :
   c_fRNG(NULL),
   c_failure_range(1, 10),
   c_failure_time(1, 1800),
   reliability_N(0),
   reliability_k(0),
   r_failure_case(1),
   s_run(1),
   s_time(0),
   s_distance(0),
   s_previous_center(0.0f, 0.0f) {
}

void CBrownianLoopFunctions::Destroy()
{
  // Close the file
  c_output.close();
}

void CBrownianLoopFunctions::Reset()
{
  s_time = 0;
  s_distance = 0;
  setFailureCasesInSwarm();
}

void CBrownianLoopFunctions::Init(TConfigurationNode& t_node)
{
   try
   {
     // Get simulation parameters from xml
     TConfigurationNode& tBrownian = GetNode(t_node, "robustness");
     GetNodeAttribute(tBrownian, "failure_case", r_failure_case);
     GetNodeAttribute(tBrownian, "k", reliability_k);     
     GetNodeAttribute(tBrownian, "N", reliability_N);
     GetNodeAttribute(tBrownian, "output_file", str_output);
     c_output.open(str_output.c_str(), std::ios_base::trunc | std::ios_base::out);
      
     // Loop thru each Epuck and randomly assign failure case 1, 2, 3
     // Reliability algorithm used k = 10% * N
     c_fRNG = CRandom::CreateRNG("argos");
     setFailureCasesInSwarm();
  }
  catch(CARGoSException& ex)
  {
    THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
  } 
}


void CBrownianLoopFunctions::PostStep()
{
  // Loop thru each Epuck to determine center of swarm distance traveled
  CVector2 total_position = CVector2(0.0f, 0.0f);
  CSpace::TMapPerType& c_epuck_entities = GetSpace().GetEntitiesByType("e-puck");
  for (CSpace::TMapPerType::iterator it = c_epuck_entities.begin(); it != c_epuck_entities.end(); ++it)
  {
    // Get handle to Epuck entity and controller
    CEPuckEntity& c_epuck = *any_cast<CEPuckEntity*>(it->second);
    CEPuckbrownian& c_controller = dynamic_cast<CEPuckbrownian&>(c_epuck.GetControllableEntity().GetController());
    s_time = GetSpace().GetSimulationClock();   

    // Add distance
    if (!c_controller.getFailureStatus())
    {
      // Check if failed
      UInt32 f_case = c_controller.getFailureCase();
      if (f_case > 0 && c_controller.getFailureTime() > s_time)
      {
        switch (f_case)
        {
          case 1: c_epuck.SetEnabled(false);  // Shutdown Epuck
                  break;
          case 2: c_epuck.GetProximitySensorEquippedEntity().SetEnabled(false);  // Shutdown proximity sensor
                  c_epuck.GetLightSensorEquippedEntity().SetEnabled(false); // Shutdown light sensor
                  c_epuck.GetRABEquippedEntity().SetEnabled(false);  // Shutdown range and bearing sensor
                  break;
          case 3: c_epuck.GetWheeledEntity().SetEnabled(false);  // shutdown motors
                  break;
          default: break;
        }
        c_controller.setFailureStatus(true);
        continue;
      }

      // Get the position of the Epuck on the ground as a CVector2
      CVector2 e_pos = c_controller.getPosition();
      total_position = total_position + e_pos;
    }
  }
  Real distance = s_previous_center - (total_position / reliability_N) ;
  s_distance = s_distance + abs(distance);  

  // Reset simulation once min 10 Epucks reached beacon, after 100 sims terminate
  if (CEPuckbrownian::getNumRobotsFinished() > 9)
  {
    // Append run# swarm-distance sim-time
    c_output << s_run << "\t" << s_distance << "\t" << s_time << std::endl;
    GetSimulator().Reset(123);
    CEPuckbrownian::resetNumRobotsFinished();
    s_run++;

    // After 100 simulations terminate
    if (s_run == 101)
    {
      GetSimulator().Terminate();
    }
  }
}

void CBrownianLoopFunctions::setFailureCasesInSwarm()
{
  // Loop thru each Epuck and randomly assign failure case 1, 2, 3
  // Reliability algorithm used k = 10% * N
  CSpace::TMapPerType& c_epuck_entities = GetSpace().GetEntitiesByType("e-puck");
  int k_val_count = 0;
  CSpace::TMapPerType::iterator it = c_epuck_entities.begin();
  while (k_val_count < reliability_k)
  {
    // Get handle to Epuck entity and controller
    CEPuckEntity& c_epuck = *any_cast<CEPuckEntity*>(it->second);
    CEPuckbrownian& c_controller = dynamic_cast<CEPuckbrownian&>(c_epuck.GetControllableEntity().GetController());

    // Randomly set fail case
    if (c_fRNG->Uniform(c_failure_range) > 5)
    {
      c_controller.setFailureCase(r_failure_case);
      c_controller.setFailureTime(c_fRNG->Uniform(c_failure_time));
      k_val_count++;
    }
    if (it == c_epuck_entities.end())
    {
      it = c_epuck_entities.begin();
    }
    else
    {
      ++it;
    }
  }
}

REGISTER_LOOP_FUNCTIONS(CBrownianLoopFunctions, "brownian_loop_functions")

