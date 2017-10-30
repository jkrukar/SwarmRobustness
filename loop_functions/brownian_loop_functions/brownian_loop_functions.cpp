#include "brownian_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <controllers/epuck_brownian/epuck_brownian.h>

CBrownianLoopFunctions::CBrownianLoopFunctions() :
   c_fRNG(NULL),
   c_failure_range(1, 10),
   c_failure_time(1, 3000),
   reliability_N(0),
   reliability_k(0),
   r_failure_case(0),
   s_run(1),
   s_time(0),
   s_distance(0),
   s_num_to_complete(0),
   s_previous_center(0.0f, 2.0f) {
}

void CBrownianLoopFunctions::Destroy()
{
  c_output.close();
}

void CBrownianLoopFunctions::Reset()
{
  s_time = 0;
  s_distance = 0;
  s_previous_center= CVector2(0.0f, 2.0f);
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
     s_num_to_complete = UInt32(0.5 * reliability_N);
      
     // Loop thru each Epuck and randomly assign failure case 1, 2, 3
     // Reliability algorithm used k = 10% * N
     c_fRNG = CRandom::CreateRNG("argos");
     setFailureCasesInSwarm();

     // Start log output to console
     std::clog << "\n\nSimulation Run# " << s_run << std::endl;
  }
  catch(CARGoSException& ex)
  {
    THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
  } 
}

void CBrownianLoopFunctions::PreStep()
{
  // Loop thru each Epuck and store position of each failure 1, 2 epucks so swarm can ignore
  // during center of swarm calculation
  CEPuckbrownian::failed_epuck_list.clear();
  CSpace::TMapPerType& c_epuck_entities = GetSpace().GetEntitiesByType("e-puck");
  for (CSpace::TMapPerType::iterator it = c_epuck_entities.begin(); it != c_epuck_entities.end(); ++it)
  {
    // Get handle to Epuck entity and controller
    CEPuckEntity& c_epuck = *any_cast<CEPuckEntity*>(it->second);
    CEPuckbrownian& c_controller = dynamic_cast<CEPuckbrownian&>(c_epuck.GetControllableEntity().GetController());

    // Get position and add to failure list
    if (c_controller.isSensorsFailed())
    {
      // Get the position of the Epuck on the ground as a CVector2
      CVector2 e_pos = c_controller.getPosition();
      CEPuckbrownian::failed_epuck_list.push_back(e_pos);
    }
  }
}

void CBrownianLoopFunctions::PostStep()
{
  s_time = GetSpace().GetSimulationClock();  

  // Loop thru each Epuck to determine center of swarm distance traveled
  CVector2 total_position = CVector2(0.0f, 0.0f);
  CSpace::TMapPerType& c_epuck_entities = GetSpace().GetEntitiesByType("e-puck");
  for (CSpace::TMapPerType::iterator it = c_epuck_entities.begin(); it != c_epuck_entities.end(); ++it)
  {
    // Get handle to Epuck entity and controller
    CEPuckEntity& c_epuck = *any_cast<CEPuckEntity*>(it->second);
    CEPuckbrownian& c_controller = dynamic_cast<CEPuckbrownian&>(c_epuck.GetControllableEntity().GetController());

    // Add distance
    if (!c_controller.isFailed())
    {
      // Check if failed
      if (c_controller.getFailureCase() > 0 && c_controller.getFailureTime() > s_time)
      {
        c_controller.generateFailure();
        continue;
      }

      // Get the position of the Epuck on the ground as a CVector2
      CVector2 e_pos = c_controller.getPosition();
      total_position = total_position + e_pos;
    }
  }
  total_position /= reliability_N;
  Real distance = (s_previous_center - total_position).Length();
  s_distance = s_distance + abs(distance);
  s_previous_center = total_position;  

  // Reset simulation once min 50% N within 200cm of beacon, terminate after 100 sims exit
  if (CEPuckbrownian::getNumRobotsFinished() > s_num_to_complete)
  {

    // Append run# swarm-distance sim-time to output file
    c_output << s_run << "\t" << s_distance << "\t" << s_time << std::endl;
    
    // Reset
    GetSimulator().Reset(1230987045);
    CEPuckbrownian::resetNumRobotsFinished();
    s_run++;

    // After 100 simulations terminate
    if (s_run > 100)
    {
      GetSimulator().Terminate();
    }
    else
    {
      std::clog << "\n\nSimulation Run# " << s_run << std::endl;
      GetSimulator().Execute();
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

    //-i Randomly set fail case
    if (c_fRNG->Uniform(c_failure_range) > 5)
    {
      c_controller.setFailureCase(r_failure_case);
      UInt32 c_fail = c_fRNG->Uniform(c_failure_time);
      c_controller.setFailureTime(c_fail);
      std::clog << "failure time=" << c_fail << std::endl;
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

