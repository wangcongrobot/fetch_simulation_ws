#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/transport/Publisher.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <unistd.h>


#include <boost/thread/mutex.hpp>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <std_srvs/Empty.h>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Pose.hh"

//#include <boost/random/beta_distribution.hpp>
//#include <boost/random/mersenne_twister.hpp>

#include <boost/math/distributions.hpp>

namespace gazebo
{
class DynamicWorldPlugin : public WorldPlugin
{
public:
  DynamicWorldPlugin() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    
    
    this->world = _world;
    GZ_ASSERT(this->world != NULL, "Got NULL world pointer!");
    this->sdf = _sdf;
    GZ_ASSERT(this->sdf != NULL, "Got NULL SDF element pointer!");
    
    // Check if Config Elements exist, otherwise they will have default value
    if (_sdf->HasElement("update_frequency"))
      this->update_frequency = _sdf->Get<double>("update_frequency");

    if (_sdf->HasElement("x_max"))
      this->x_max = _sdf->Get<double>("x_max");
    if (_sdf->HasElement("x_min"))
      this->x_min = _sdf->Get<double>("x_min");
    if (_sdf->HasElement("y_max"))
      this->y_max = _sdf->Get<double>("y_max");
    if (_sdf->HasElement("y_min"))
      this->y_min = _sdf->Get<double>("y_min");
    if (_sdf->HasElement("z_max"))
      this->z_max = _sdf->Get<double>("z_max");
    if (_sdf->HasElement("z_min"))
      this->z_min = _sdf->Get<double>("z_min");
      
    if (_sdf->HasElement("x_max_distractors"))
      this->x_max_distractors = _sdf->Get<double>("x_max_distractors");
    if (_sdf->HasElement("x_min_distractors"))
      this->x_min_distractors = _sdf->Get<double>("x_min_distractors");
    if (_sdf->HasElement("y_max_distractors"))
      this->y_max_distractors = _sdf->Get<double>("y_max_distractors");
    if (_sdf->HasElement("y_min_distractors"))
      this->y_min_distractors = _sdf->Get<double>("y_min_distractors");
    if (_sdf->HasElement("z_max_distractors"))
      this->z_max_distractors = _sdf->Get<double>("z_max_distractors");
    if (_sdf->HasElement("z_min_distractors"))
      this->z_min_distractors = _sdf->Get<double>("z_min_distractors");

    if (_sdf->HasElement("z_increment_percentage"))
      this->z_increment_percentage = _sdf->Get<double>("z_increment_percentage");


    if (_sdf->HasElement("roll_max"))
      this->roll_max = _sdf->Get<double>("roll_max");
    if (_sdf->HasElement("roll_min"))
      this->roll_min = _sdf->Get<double>("roll_min");
    if (_sdf->HasElement("pitch_max"))
      this->pitch_max = _sdf->Get<double>("pitch_max");
    if (_sdf->HasElement("pitch_min"))
      this->pitch_min = _sdf->Get<double>("pitch_min");
    if (_sdf->HasElement("yaw_max"))
      this->yaw_max = _sdf->Get<double>("yaw_max");
    if (_sdf->HasElement("yaw_min"))
      this->yaw_min = _sdf->Get<double>("yaw_min");


    // Lights
    if (_sdf->HasElement("light_x_max"))
      this->light_x_max = _sdf->Get<double>("light_x_max");
    if (_sdf->HasElement("light_x_min"))
      this->light_x_min = _sdf->Get<double>("light_x_min");
    if (_sdf->HasElement("light_y_max"))
      this->light_y_max = _sdf->Get<double>("light_y_max");
    if (_sdf->HasElement("light_y_min"))
      this->light_y_min = _sdf->Get<double>("light_y_min");
    if (_sdf->HasElement("light_z_max"))
      this->light_z_max = _sdf->Get<double>("light_z_max");
    if (_sdf->HasElement("light_z_min"))
      this->light_z_min = _sdf->Get<double>("light_z_min");


    if (_sdf->HasElement("light_roll_max"))
      this->light_roll_max = _sdf->Get<double>("light_roll_max");
    if (_sdf->HasElement("light_roll_min"))
      this->light_roll_min = _sdf->Get<double>("light_roll_min");
    if (_sdf->HasElement("light_pitch_max"))
      this->light_pitch_max = _sdf->Get<double>("light_pitch_max");
    if (_sdf->HasElement("light_pitch_min"))
      this->light_pitch_min = _sdf->Get<double>("light_pitch_min");
    if (_sdf->HasElement("light_yaw_max"))
      this->light_yaw_max = _sdf->Get<double>("light_yaw_max");
    if (_sdf->HasElement("light_yaw_min"))
      this->light_yaw_min = _sdf->Get<double>("light_yaw_min");

    // Lights
    if (_sdf->HasElement("cam_x_max"))
      this->cam_x_max = _sdf->Get<double>("cam_x_max");
    if (_sdf->HasElement("cam_x_min"))
      this->cam_x_min = _sdf->Get<double>("cam_x_min");
    if (_sdf->HasElement("cam_y_max"))
      this->cam_y_max = _sdf->Get<double>("cam_y_max");
    if (_sdf->HasElement("cam_y_min"))
      this->cam_y_min = _sdf->Get<double>("cam_y_min");
    if (_sdf->HasElement("cam_z_max"))
      this->cam_z_max = _sdf->Get<double>("cam_z_max");
    if (_sdf->HasElement("cam_z_min"))
      this->cam_z_min = _sdf->Get<double>("cam_z_min");


    if (_sdf->HasElement("cam_roll_max"))
      this->cam_roll_max = _sdf->Get<double>("cam_roll_max");
    if (_sdf->HasElement("cam_roll_min"))
      this->cam_roll_min = _sdf->Get<double>("cam_roll_min");
    if (_sdf->HasElement("cam_pitch_max"))
      this->cam_pitch_max = _sdf->Get<double>("cam_pitch_max");
    if (_sdf->HasElement("cam_pitch_min"))
      this->cam_pitch_min = _sdf->Get<double>("cam_pitch_min");
    if (_sdf->HasElement("cam_yaw_max"))
      this->cam_yaw_max = _sdf->Get<double>("cam_yaw_max");
    if (_sdf->HasElement("cam_yaw_min"))
      this->cam_yaw_min = _sdf->Get<double>("cam_yaw_min");
    
    if (_sdf->HasElement("random_distribution_type"))
    {
      this->random_distribution_type = _sdf->Get<std::string>("random_distribution_type");
    }else
    {
      this->random_distribution_type = "uniform";
    }
    
    if (_sdf->HasElement("alpha"))
    {
      this->alpha = _sdf->Get<double>("alpha");
    }else
    {
      this->alpha = 0.0;
    }
    
    
    if (_sdf->HasElement("beta"))
    {
      this->beta = _sdf->Get<double>("beta");
    }else
    {
      this->beta = 0.0;
    }
    
    // yes if we want it to be active, no if we dont want elements of the scene to change colours and lights
    if (_sdf->HasElement("colour_randomiser_active"))
      this->colour_randomiser_active = _sdf->Get<std::string>("colour_randomiser_active");
      
    
    if (_sdf->HasElement("model_to_track_name"))
      this->model_to_track_name = _sdf->Get<std::string>("model_to_track_name");
      
    std::string models_to_ignore_colours_list = "";
    std::string models_to_ignore_movement_list = "";
    char delimiter = ',';
    
    if (_sdf->HasElement("model_to_ignore_randcolours"))
      
      models_to_ignore_colours_list = _sdf->Get<std::string>("model_to_ignore_randcolours");

      this->model_to_ignore_randcolours = this->split(models_to_ignore_colours_list, delimiter);
      
      for ( auto &model_name_x : this->model_to_ignore_randcolours )
      {
          ROS_WARN("Ignore Colours model_name_x==%s", model_name_x.c_str());
      }
      
      if (this->check_if_vector_contains_string(this->model_to_ignore_randcolours,"rgb_cam"))
      {
        ROS_WARN("Colours String Vector Finder Works!");
      }
      
    
    if (_sdf->HasElement("model_to_ignore_movement"))
      
      models_to_ignore_movement_list = _sdf->Get<std::string>("model_to_ignore_movement");
      
      this->model_to_ignore_movement = this->split(models_to_ignore_movement_list, delimiter);
      
      for ( auto &model_name_x : this->model_to_ignore_movement )
      {
          ROS_WARN("Ignore MOV model_name_x==%s", model_name_x.c_str());
      }
      
      if (this->check_if_vector_contains_string(this->model_to_ignore_movement,"rgb_cam"))
      {
        ROS_INFO("Movement String Vector Finder Works!");
      }

    

    // Create the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    // Init Gazebo Publishers
    this->visPub = this->node->Advertise<gazebo::msgs::Visual>("/gazebo/default/visual");
    ROS_DEBUG("Waiting for connection of Visual Topic...");
    this->visPub->WaitForConnection();
    ROS_DEBUG("Waiting for connection of Visual Topic...DONE");

    this->lightPub = this->node->Advertise<gazebo::msgs::Light>("/gazebo/default/light/modify");
    ROS_DEBUG("Waiting for connection of Light Topic...");
    this->lightPub->WaitForConnection();
    ROS_DEBUG("Waiting for connection of Light Topic...DONE");

    // We wait for all system to be ready an amount of seconds
    float seconds_to_wait = 5.0;
    this->WaitForseconds(seconds_to_wait);

    // Update Time Init
    this->old_secs =this->world->GetSimTime().Float();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DynamicWorldPlugin::OnUpdate, this));


    // ROS Stuff Now
    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("earthquake_rosnode"));

    serviceName = "/dynamic_world_service";
    // advertise services for calibration and bias setting
    if (!serviceName.empty())
        srv_ = this->rosNode->advertiseService(serviceName, &DynamicWorldPlugin::ServiceCallback, this);
    // start custom queue for imu
    callback_queue_thread_ = boost::thread( boost::bind( &DynamicWorldPlugin::CallbackQueueThread,this ) );


    ROS_DEBUG("Dynamic World Ready...The Only Thing That Is Constant Is Change - Quote by Heraclitus");
  }
  
  
  std::vector<std::string> split(const std::string& s, char delimiter)
  {
     std::vector<std::string> tokens;
     std::string token;
     std::istringstream tokenStream(s);
     while (std::getline(tokenStream, token, delimiter))
     {
        tokens.push_back(token);
     }
     return tokens;
  }
  
  
  bool check_if_vector_contains_string(const std::vector<std::string> vector_obj, const std::string& s)
  {
    if (std::find(vector_obj.begin(), vector_obj.end(), s) != vector_obj.end())
    {
      return true;
    }else
    {
      return false;
    }
     
  }


  void Reset()
  {
    this->reseting_plugin = true;
    ROS_INFO("Reseted the simulation world, we Restart the time clock");
    // Update Time Init
    this->old_secs = 0.0;
    double new_secs = 0.0;
    double delta = -1.0;

    while (delta < 0.0)
    {
      // We change Direction
      ROS_INFO("Waiting until Clock is reseted and delta is not negative > Update delta=%f, new_secs=%f", delta,  new_secs);
      new_secs = this->world->GetSimTime().Float();
      delta = new_secs - this->old_secs;
      ROS_INFO("Updated until Clock is reseted > Update delta=%f, new_secs=%f", delta,  new_secs);

    }

    this->reseting_plugin = false;

  }


  // Called by the world update start event
  public: void OnUpdate()
  {


    if (this->reseting_plugin)
    {
        ROS_WARN("Reseting in Process, please wait...");
    }else
    {
        // TODO: Check what is necessary now here
        double new_secs =this->world->GetSimTime().Float();
        double delta = new_secs - this->old_secs;

        double max_delta = 0.0;

        if (this->update_frequency != 0.0)
        {
          max_delta = 1.0 / this->update_frequency;
        }

        if (delta > max_delta && delta != 0.0)
        {
          // We update the Old Time variable.
          this->old_secs = new_secs;
        }
    }



  }

  void WaitForseconds(float seconds_to_wait)
  {
    unsigned int microseconds;
    microseconds = seconds_to_wait * 1e6;
    ROS_DEBUG("Waiting for %f seconds",seconds_to_wait);
    usleep(microseconds);
    ROS_DEBUG("Done waiting...");

  }

  void RandomiseWorldModels()
  {
    
    if (this->colour_randomiser_active == "yes")
    {
      this->UpdateWorldModels();
      //this->OutputWorldModelsData();
    }else
    {
      ROS_WARN("The Randomiser is Disconected, no changes in the models will be made. Check the plugin input colour_randomiser_active");
    }

    this->MoveModels();
    this->MoveCamera();

    // Change Colours to Random
    for (auto const& x : this->modelIDToName)
    {
        //ROS_DEBUG("ModelID=%i, Name=%s", x.first, x.second.c_str());
        std::string model_name = x.second;
        //std::string model_name = "demo_cube_X";
        int new_R = 0;
        int new_G = 0;
        int new_B = 0;
        int new_ALFA = 255;

        new_R = rand() % 256;
        new_G = rand() % 256;
        new_B = rand() % 256;
        new_ALFA = 255;

        ROS_DEBUG("RGB-RANDOM=[%i,%i,%i,%i], model=%s", new_R,new_G,new_B,new_ALFA, model_name.c_str());
        bool result = false;
        
        
        //if (model_name != "rgb_cam" && model_name!="fetch")
        if (!this->check_if_vector_contains_string(this->model_to_ignore_randcolours,model_name))
        {
          result = this->ChangeModelColour(model_name, new_R, new_G, new_B, new_ALFA);
        }
    }

  }


  void RandomiseWorldLights()
  {    
    
    this->UpdateLightsWorld();
    this->OutputWorldLightsData();

    for (auto const& x : this->lightIDToName)
    {
        std::string model_name = x.second;
        bool result = false;
        result = this->ChangeLight(model_name);
    }




  }


  void UpdateWorldModels()
  {
    // We have to clear the Maps otherwise it wont reflect the changes
    //ROS_DEBUG("Clearing modelIDToName MAP...");
    this->modelIDToName.clear();
    //this->OutputWorldModelsData();
    //ROS_DEBUG("Clearing modelIDToName MAP...DONE");

    // Initialize color map.
    for (auto model : this->world->GetModels())
    {
      // A model might have multiple links, a link might have multiple visuals
      // with different colors
      // whatever man, this is probably not a typical case for teams

      // numVecs: the number of vectors we average over.
      // For a model with m visuals, numVecs will be 2*m
      // because each visual has two color vectors, ambient and diffuse
      int numVecs = 0;
      double r, g, b, a;
      r = g = b = a = 0;

      for (auto link : model->GetLinks())
      {
        // Get all the visuals, sdf is imported by Plugin.hh somehow , but not the Link Definition
        sdf::ElementPtr linkSDF = link->GetSDF();
        GZ_ASSERT(linkSDF != NULL, "Got link with NULL SDF pointer in init");
        if (linkSDF->HasElement("visual"))
        {
          for (sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
               visualSDF; visualSDF = linkSDF->GetNextElement("visual"))
          {
            GZ_ASSERT(visualSDF->HasAttribute("name"),
                "Malformed visual element!");
            if (!visualSDF->HasElement("material"))
              continue;
            sdf::ElementPtr materialSDF = visualSDF->GetElement("material");
            if (!materialSDF->HasElement("ambient") &&
                !materialSDF->HasElement("diffuse"))
            {
              continue;
            }
            gazebo::common::Color ambient =
                materialSDF->GetElement("ambient")->Get<gazebo::common::Color>();
            gazebo::common::Color diffuse =
                materialSDF->GetElement("diffuse")->Get<gazebo::common::Color>();

            r += ambient.r;
            g += ambient.g;
            b += ambient.b;
            a += ambient.a;

            r += diffuse.r;
            g += diffuse.g;
            b += diffuse.b;
            a += diffuse.a;
            numVecs+=2;
          }
        }
      }
      if (numVecs == 0)
        numVecs = 1;
      gazebo::common::Color color(r / numVecs, g / numVecs, b / numVecs,
          a / numVecs);
      this->lastKnownColors[model->GetId()] = color;
      this->modelIDToName[model->GetId()] = model->GetName();
    }
  }


  void UpdateLightsWorld()
  {
    // We have to clear the Maps otherwise it wont reflect the changes
    //ROS_DEBUG("Clearing lightIDToName MAP...");
    this->lightIDToName.clear();
    //this->OutputWorldLightsData();
    //ROS_DEBUG("Clearing lightIDToName MAP...DONE");

    auto lights_v = this->world->Lights();
    int size = static_cast<int>(lights_v.size());
    //ROS_DEBUG("LightVector SIZE=%i", size);

    for (auto light : this->world->Lights())
    {
      // light should be a LightPtr
      this->lightIDToName[light->GetId()] = light->GetName();


      //ROS_DEBUG("LightName=%s", light->GetName().c_str());
    }
  }

  void OutputWorldModelsData()
  {
    ROS_DEBUG("Start OutputWorldModelsData...");

    for (auto const& x : this->modelIDToName)
    {
        ROS_DEBUG("ModelID=%i, Name=%s", x.first, x.second.c_str());
    }

    ROS_DEBUG("END OutputWorldModelsData...");

  }


  void OutputWorldLightsData()
  {
    ROS_DEBUG("Start OutputWorldLightsData...");

    for (auto const& x : this->lightIDToName)
    {
        ROS_DEBUG("LightID=%i, Name=%s", x.first, x.second.c_str());
    }

    ROS_DEBUG("END OutputWorldLightsData...");

  }

  bool ChangeModelColour(std::string model_name, int new_R, int new_G, int new_B, int new_ALFA)
  {

    //ROS_DEBUG("START ChangeModelColour...");
    gazebo::physics::ModelPtr model = this->world->GetModel(model_name);

    if (!model)
    {
      ROS_ERROR("Model named [%s] could not be found", model_name.c_str());
      return false;
    }else
    {
      //ROS_DEBUG("Model %s Found in World", model_name.c_str());
    }


    gazebo::common::Color newColor(new_R, new_G, new_B, new_ALFA);

    gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
    gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);

    //ROS_DEBUG("Created ColorMsg and diffuseMsg");




    for (auto link : model->GetLinks())
    {
      // Get all the visuals
      sdf::ElementPtr linkSDF = link->GetSDF();

      if (!linkSDF)
      {
        ROS_ERROR("Link had NULL SDF");
        return false;
      }else
      {
        ROS_DEBUG("Link [%s] SDF found", link->GetName().c_str());
      }



      if (linkSDF->HasElement("visual"))
      {


        for (sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
             visualSDF; visualSDF = linkSDF->GetNextElement("visual"))
        {
          ROS_DEBUG("Processing Each Visual Element of the Link...");

          if (visualSDF->HasAttribute("name"))
          {
            ROS_DEBUG("visualSDF HAS the attribute name");
          }else
          {
            ROS_ERROR("attribute name in visualSDF NOT FOUND");
          }


          //GZ_ASSERT(visualSDF->HasAttribute("name"), "Malformed visual element!");

          std::string visualName = visualSDF->Get<std::string>("name");

          gazebo::msgs::Visual visMsg;

          visMsg = link->GetVisualMessage(visualName);

          ROS_DEBUG("visualName=%s",visualName.c_str());
          ROS_DEBUG("link->GetScopedName()=%s",link->GetScopedName().c_str());

          if ((!visMsg.has_material()) || visMsg.mutable_material() == NULL)
          {
            gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
            visMsg.set_allocated_material(materialMsg);
          }else{ROS_ERROR("visMsg doesnt have material or mutable_material");}


          gazebo::msgs::Material *materialMsg = visMsg.mutable_material();

          ROS_DEBUG("materialMsg Created");



          if (materialMsg->has_ambient())
          {
            materialMsg->clear_ambient();
          }
          materialMsg->set_allocated_ambient(colorMsg);
          if (materialMsg->has_diffuse())
          {
            materialMsg->clear_diffuse();
          }
          visMsg.set_name(link->GetScopedName());
          //std::string visual_element_string = link->GetScopedName() + "::" + visualName;
          //visMsg.set_name(visual_element_string);
          visMsg.set_parent_name(model->GetScopedName());
          materialMsg->set_allocated_diffuse(diffuseMsg);

          ROS_DEBUG("visMsg READY TO PUBLISH");
          this->visPub->Publish(visMsg);
          ROS_DEBUG("visMsg PUBLISH DONE");
        }

      }else{ROS_ERROR("Link has no Element visual");}

    }

    this->lastKnownColors[model->GetId()] = newColor;
    ROS_DEBUG("END ChangeModelColour...");
    return true;
  }


  void MoveModels()
  {

    int models_counter = 0;
    for (auto model : this->world->GetModels())
    {
        std::string model_name = model->GetName();

        
        
        //if (model_name != "rgb_cam" && model_name != "demo_table1" && model_name != "ground_plane" && model_name != "fetch" && model_name != "rubbish_bin1")
        if (!this->check_if_vector_contains_string(this->model_to_ignore_movement,model_name))
        {
            ROS_DEBUG("Moving model=%s",model_name.c_str());
            
            float x_pos_rand;
            float y_pos_rand;
            float z_pos_rand;
            
            float roll_rand;
            float pitch_rand;
            float yaw_rand;

            float x_min_value;
            float x_max_value;
            float y_min_value;
            float y_max_value;
            float z_min_value;
            float z_max_value;

            // We check if its the tracker model or the rest.
            if (model_name == this->model_to_track_name)
            {
              x_min_value = this->x_min;
              x_max_value = this->x_max;
              
              y_min_value = this->y_min;
              y_max_value = this->y_max;
              
              z_min_value = this->z_min;
              z_max_value = this->z_max;
              
              z_pos_rand = RandomFloat(z_min_value, z_max_value);
            
            }else
            {
              
              x_min_value = this->x_min_distractors;
              x_max_value = this->x_max_distractors;
              
              y_min_value = this->y_min_distractors;
              y_max_value = this->y_max_distractors;
              
              z_min_value = this->z_min_distractors;
              z_max_value = this->z_max_distractors;
              
              z_pos_rand = RandomFloat(z_min_value, z_max_value) + models_counter * this->z_min * this->z_increment_percentage;
            
              
            }
            
            
            x_pos_rand = RandomFloat(x_min_value, x_max_value);
            
            // We generate y values random that follow a beta 0.5 0.5 distribution
            // to give more oportunities to have the object in the limits of the table.
            if (this->random_distribution_type == "beta_distribution")
            {
              y_pos_rand = this->RandomFloatBeta(y_min_value, y_max_value);
            }else
            {
              ROS_WARN("The Distribution is Not BETA.==%s", this->random_distribution_type.c_str());
              y_pos_rand = RandomFloat(y_min_value, y_max_value);
            }
            
            
            roll_rand = RandomFloat(this->roll_min, this->roll_max);
            pitch_rand = RandomFloat(this->pitch_min, this->pitch_max);
            yaw_rand = RandomFloat(this->yaw_min, this->yaw_max);

            ROS_DEBUG("POSE-RANDOM[X,Y,Z,Roll,Pitch,Yaw=[%f,%f,%f,%f,%f,%f], model=%s", x_pos_rand,y_pos_rand,z_pos_rand,roll_rand,pitch_rand,yaw_rand,model_name.c_str());
            gazebo::math::Pose initPose(math::Vector3(x_pos_rand, y_pos_rand, z_pos_rand), math::Quaternion(roll_rand, pitch_rand, yaw_rand));
            model->SetWorldPose(initPose);
            ROS_DEBUG("Moving model=%s....END",model_name.c_str());

            models_counter++;
        }

    }

  }


  void MoveCamera()
  {

    for (auto model : this->world->GetModels())
    {
        std::string model_name = model->GetName();

        float x_pos_rand = 0.0;
        float y_pos_rand = 0.0;
        float z_pos_rand = 0.0;
        float roll_rand = 0.0;
        float pitch_rand = 0.0;
        float yaw_rand = 0.0;

        if (model_name == "rgb_cam")
        {
            ROS_DEBUG("Moving model=%s",model_name.c_str());

            float x_pos_rand = RandomFloat(this->cam_x_min, this->cam_x_max);
            float y_pos_rand = RandomFloat(this->cam_y_min, this->cam_y_max);
            float z_pos_rand = RandomFloat(this->cam_z_min, this->cam_z_max);
            float roll_rand = RandomFloat(this->cam_roll_min, this->cam_roll_max);
            float pitch_rand = RandomFloat(this->cam_pitch_min, this->cam_pitch_max);
            float yaw_rand = RandomFloat(this->cam_yaw_min, this->cam_yaw_max);

            ROS_WARN("CAM-RANDOM[X,Y,Z,Roll,Pitch,Yaw=[%f,%f,%f,%f,%f,%f], model=%s", x_pos_rand,y_pos_rand,z_pos_rand,roll_rand,pitch_rand,yaw_rand,model_name.c_str());
            gazebo::math::Pose initPose(math::Vector3(x_pos_rand, y_pos_rand, z_pos_rand), math::Quaternion(roll_rand, pitch_rand, yaw_rand));
            model->SetWorldPose(initPose);
            ROS_WARN("Moving camera=%s....END",model_name.c_str());
        }

    }

  }


  bool ChangeLight(std::string light_name)
  {
    ROS_DEBUG("Changed Light=%s...",light_name.c_str());

    float diffuse_R = RandomFloat(0.0, 1.0);
    float diffuse_G = RandomFloat(0.0, 1.0);
    float diffuse_B = RandomFloat(0.0, 1.0);
    float diffuse_A = RandomFloat(0.0, 1.0);

    float specular_R = RandomFloat(0.0, 1.0);
    float specular_G = RandomFloat(0.0, 1.0);
    float specular_B = RandomFloat(0.0, 1.0);
    float specular_A = RandomFloat(0.0, 1.0);

    float x_pos_rand = RandomFloat(this->light_x_min, this->light_x_max);
    float y_pos_rand = RandomFloat(this->light_y_min, this->light_y_max);
    float z_pos_rand = RandomFloat(this->light_z_min, this->light_z_max);
    float roll_rand = RandomFloat(this->light_roll_min, this->light_roll_max);
    float pitch_rand = RandomFloat(this->light_pitch_min, this->light_pitch_max);
    float yaw_rand = RandomFloat(this->light_yaw_min, this->light_yaw_max);


    gazebo::msgs::Light msg;
    msg.set_name(light_name);
    
    if (this->colour_randomiser_active == "yes")
    {
      gazebo::msgs::Set(msg.mutable_diffuse(), common::Color(diffuse_R, diffuse_G, diffuse_B, diffuse_A));
      gazebo::msgs::Set(msg.mutable_specular(), common::Color(specular_R, specular_G, specular_B, specular_A));
    }else
    {
      ROS_WARN("The Randomiser is Disconected, no changes in the lights will be made. Check the plugin input colour_randomiser_active");
    }

    gazebo::msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(x_pos_rand, y_pos_rand, z_pos_rand, roll_rand, pitch_rand, yaw_rand));
    this->lightPub->Publish(msg);

    ROS_DEBUG("Changed Light=%s...END",light_name.c_str());
    return true;
  }




  float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
  }
  
  float RandomFloatBeta(float a, float b) {
    // https://www.boost.org/doc/libs/1_49_0/libs/math/doc/sf_and_dist/html/math_toolkit/dist/dist_ref/dists/beta_dist.html
    // https://en.wikipedia.org/wiki/Bayesian_inference

    // Here we test random poses with Beta-distribution
    float randFromUnif = RandomFloat(a, b);
    //parameters and the random value on (0,1) you drew
    
    boost::math::beta_distribution<> dist(this->alpha, this->beta);
    // Linear interpolation quantile.
    float randFromDist = quantile(dist, randFromUnif);
    ROS_WARN("BETA RAND VALUE=%f....",randFromDist);
    
    return randFromDist;
  }


  bool ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
  {
      boost::mutex::scoped_lock scoped_lock(lock);
      ROS_WARN("You Called The service!");

      this->RandomiseWorldModels();
      this->RandomiseWorldLights();

      return true;
  }

  void CallbackQueueThread()
  {
      static const double timeout = 0.01;

      while (this->rosNode->ok())
      {
        this->callback_queue_.callAvailable(ros::WallDuration(timeout));
      }
  }
  
  
  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  
  /// \brief World pointer.
  protected: gazebo::physics::WorldPtr world;
  /// \brief SDF pointer.
  protected: sdf::ElementPtr sdf;
  /// \brief Maps model IDs to colors
  private: std::map<int, gazebo::common::Color> lastKnownColors;
  /// \brief Maps model IDs to ModelNames
  private: std::map<int, std::string> modelIDToName;
  
  /// \brief Maps light IDs to LightNames
  private: std::map<int, std::string> lightIDToName;
  
  
  // Time Memory
  double old_secs;
  // Frequency of update
  double update_frequency = 0.2;
  // Range of Random Positions of Object on Table
  double x_max = 1.0;
  double x_min = 0.0;
  double y_max = 1.0;
  double y_min = 0.0;
  double z_max = 1.0;
  double z_min = 0.0;
  
  double x_max_distractors = 1.0;
  double x_min_distractors = 0.0;
  double y_max_distractors = 1.0;
  double y_min_distractors = 0.0;
  double z_max_distractors = 1.0;
  double z_min_distractors = 0.0;

  double z_increment_percentage = 0.01;

  double roll_max = 1.0;
  double roll_min = 0.0;
  double pitch_max = 1.0;
  double pitch_min = 0.0;
  double yaw_max = 1.0;
  double yaw_min = 0.0;


  double light_x_max = 1.0;
  double light_x_min = 0.0;
  double light_y_max = 1.0;
  double light_y_min = 0.0;
  double light_z_max = 1.0;
  double light_z_min = 0.0;

  double light_roll_max = 1.0;
  double light_roll_min = 0.0;
  double light_pitch_max = 1.0;
  double light_pitch_min = 0.0;
  double light_yaw_max = 1.0;
  double light_yaw_min = 0.0;


  double cam_x_max = 1.0;
  double cam_x_min = 0.0;
  double cam_y_max = 1.0;
  double cam_y_min = 0.0;
  double cam_z_max = 1.0;
  double cam_z_min = 0.0;

  double cam_roll_max = 1.0;
  double cam_roll_min = 0.0;
  double cam_pitch_max = 1.0;
  double cam_pitch_min = 0.0;
  double cam_yaw_max = 1.0;
  double cam_yaw_min = 0.0;
  
  std::string random_distribution_type = "uniform";
  double alpha = 2.0;
  double beta = 2.0;
  
  std::string colour_randomiser_active = "yes";
  
  std::string model_to_track_name = "demo_spam1";
  
  std::vector<std::string> model_to_ignore_randcolours;
  std::vector<std::string> model_to_ignore_movement;

  // Reseting Flag
  bool reseting_plugin = false;
  
  /// \brief A node used for transport
  private: transport::NodePtr node;
  /// \brief For publishing visual messages to ~/visual
  private: transport::PublisherPtr visPub;
  // Publisher for light modification
  transport::PublisherPtr lightPub;

  // ROS Stuff
  /// \brief A node use for ROS transport
  private: std::unique_ptr<ros::NodeHandle> rosNode;
  private: ros::ServiceServer srv_;
  private: ros::CallbackQueue callback_queue_;
  private: boost::thread callback_queue_thread_;
  private: std::string serviceName;
  /// \brief A mutex to lock access to fields that are used in message callbacks
  boost::mutex lock;
  
  
};
GZ_REGISTER_WORLD_PLUGIN(DynamicWorldPlugin)
}