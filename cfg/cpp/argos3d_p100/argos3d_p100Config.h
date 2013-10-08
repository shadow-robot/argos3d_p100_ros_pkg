//#line 2 "/opt/ros/hydro/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the argos3d_p100 package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __argos3d_p100__ARGOS3D_P100CONFIG_H__
#define __argos3d_p100__ARGOS3D_P100CONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace argos3d_p100
{
  class argos3d_p100ConfigStatics;
  
  class argos3d_p100Config
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(argos3d_p100Config &config, const argos3d_p100Config &max, const argos3d_p100Config &min) const = 0;
      virtual void calcLevel(uint32_t &level, const argos3d_p100Config &config1, const argos3d_p100Config &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, argos3d_p100Config &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const argos3d_p100Config &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, argos3d_p100Config &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const argos3d_p100Config &config) const = 0;
      virtual void getValue(const argos3d_p100Config &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T argos3d_p100Config::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (argos3d_p100Config::* field);

      virtual void clamp(argos3d_p100Config &config, const argos3d_p100Config &max, const argos3d_p100Config &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const argos3d_p100Config &config1, const argos3d_p100Config &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, argos3d_p100Config &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const argos3d_p100Config &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, argos3d_p100Config &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const argos3d_p100Config &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const argos3d_p100Config &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, argos3d_p100Config &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); i++)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;
        
        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++) 
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }
      
      virtual void updateParams(boost::any &cfg, argos3d_p100Config &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++) 
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<argos3d_p100Config::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(argos3d_p100Config &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = params.begin(); i != params.end(); i++)
      {
        boost::any val;
        (*i)->getValue(config, val);

        if("Integration_Time"==(*i)->name){Integration_Time = boost::any_cast<int>(val);}
        if("Modulation_Frequency"==(*i)->name){Modulation_Frequency = boost::any_cast<int>(val);}
        if("At_Least"==(*i)->name){At_Least = boost::any_cast<bool>(val);}
        if("At_Most"==(*i)->name){At_Most = boost::any_cast<bool>(val);}
        if("Statistical_Noise_Filter_On"==(*i)->name){Statistical_Noise_Filter_On = boost::any_cast<bool>(val);}
        if("Noise_Filtering_NoOfNeighbours"==(*i)->name){Noise_Filtering_NoOfNeighbours = boost::any_cast<int>(val);}
        if("Std_Dev_Mul_Threshold"==(*i)->name){Std_Dev_Mul_Threshold = boost::any_cast<double>(val);}
        if("Amplitude_Filter_On"==(*i)->name){Amplitude_Filter_On = boost::any_cast<bool>(val);}
        if("Amplitude_Threshold"==(*i)->name){Amplitude_Threshold = boost::any_cast<double>(val);}
      }
    }

    int Integration_Time;
int Modulation_Frequency;
bool At_Least;
bool At_Most;
bool Statistical_Noise_Filter_On;
int Noise_Filtering_NoOfNeighbours;
double Std_Dev_Mul_Threshold;
bool Amplitude_Filter_On;
double Amplitude_Threshold;

    bool state;
    std::string name;

    
}groups;



//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      int Integration_Time;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      int Modulation_Frequency;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool At_Least;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool At_Most;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool Statistical_Noise_Filter_On;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      int Noise_Filtering_NoOfNeighbours;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double Std_Dev_Mul_Threshold;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool Amplitude_Filter_On;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double Amplitude_Threshold;
//#line 255 "/opt/ros/hydro/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("argos3d_p100Config::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const argos3d_p100Config &__max__ = __getMax__();
      const argos3d_p100Config &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const argos3d_p100Config &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const argos3d_p100Config &__getDefault__();
    static const argos3d_p100Config &__getMax__();
    static const argos3d_p100Config &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const argos3d_p100ConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void argos3d_p100Config::ParamDescription<std::string>::clamp(argos3d_p100Config &config, const argos3d_p100Config &max, const argos3d_p100Config &min) const
  {
    return;
  }

  class argos3d_p100ConfigStatics
  {
    friend class argos3d_p100Config;
    
    argos3d_p100ConfigStatics()
    {
argos3d_p100Config::GroupDescription<argos3d_p100Config::DEFAULT, argos3d_p100Config> Default("Default", "", 0, 0, true, &argos3d_p100Config::groups);
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Integration_Time = 238;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Integration_Time = 5000;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Integration_Time = 250;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<int>("Integration_Time", "int", 0, "Integration time(in msec) for the sensor", "", &argos3d_p100Config::Integration_Time)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<int>("Integration_Time", "int", 0, "Integration time(in msec) for the sensor", "", &argos3d_p100Config::Integration_Time)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Modulation_Frequency = 18000000;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Modulation_Frequency = 23000000;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Modulation_Frequency = 20000000;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<int>("Modulation_Frequency", "int", 0, "Set the modulation frequency(Hz) of the sensor. By Default the frequency closest to the given value will be set.", "", &argos3d_p100Config::Modulation_Frequency)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<int>("Modulation_Frequency", "int", 0, "Set the modulation frequency(Hz) of the sensor. By Default the frequency closest to the given value will be set.", "", &argos3d_p100Config::Modulation_Frequency)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.At_Least = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.At_Least = 1;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.At_Least = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("At_Least", "bool", 0, "Modulation Frequency no less than the entered frequency will be set", "", &argos3d_p100Config::At_Least)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("At_Least", "bool", 0, "Modulation Frequency no less than the entered frequency will be set", "", &argos3d_p100Config::At_Least)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.At_Most = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.At_Most = 1;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.At_Most = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("At_Most", "bool", 0, "Modulation Frequency no more than the entered frequency will be set", "", &argos3d_p100Config::At_Most)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("At_Most", "bool", 0, "Modulation Frequency no more than the entered frequency will be set", "", &argos3d_p100Config::At_Most)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Statistical_Noise_Filter_On = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Statistical_Noise_Filter_On = 1;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Statistical_Noise_Filter_On = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("Statistical_Noise_Filter_On", "bool", 0, "Whether to apply statistical noise filter from pcl or not.", "", &argos3d_p100Config::Statistical_Noise_Filter_On)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("Statistical_Noise_Filter_On", "bool", 0, "Whether to apply statistical noise filter from pcl or not.", "", &argos3d_p100Config::Statistical_Noise_Filter_On)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Noise_Filtering_NoOfNeighbours = 1;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Noise_Filtering_NoOfNeighbours = 200;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Noise_Filtering_NoOfNeighbours = 30;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<int>("Noise_Filtering_NoOfNeighbours", "int", 0, "No of neighbours to be considered for applying statistical noise reduction", "", &argos3d_p100Config::Noise_Filtering_NoOfNeighbours)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<int>("Noise_Filtering_NoOfNeighbours", "int", 0, "No of neighbours to be considered for applying statistical noise reduction", "", &argos3d_p100Config::Noise_Filtering_NoOfNeighbours)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Std_Dev_Mul_Threshold = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Std_Dev_Mul_Threshold = 10.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Std_Dev_Mul_Threshold = 0.4;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<double>("Std_Dev_Mul_Threshold", "double", 0, "Standard Deviation Multiplier Threshold for applying statistical noise reduction", "", &argos3d_p100Config::Std_Dev_Mul_Threshold)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<double>("Std_Dev_Mul_Threshold", "double", 0, "Standard Deviation Multiplier Threshold for applying statistical noise reduction", "", &argos3d_p100Config::Std_Dev_Mul_Threshold)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Amplitude_Filter_On = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Amplitude_Filter_On = 1;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Amplitude_Filter_On = 0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("Amplitude_Filter_On", "bool", 0, "Whether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out", "", &argos3d_p100Config::Amplitude_Filter_On)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<bool>("Amplitude_Filter_On", "bool", 0, "Whether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out", "", &argos3d_p100Config::Amplitude_Filter_On)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.Amplitude_Threshold = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.Amplitude_Threshold = 2500.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.Amplitude_Threshold = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<double>("Amplitude_Threshold", "double", 0, "What should be the amplitude filter threshold. Image pixels with lesser aplitude values will be filtered out. Amplitude Filter Status should be true to use this filter.", "", &argos3d_p100Config::Amplitude_Threshold)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(argos3d_p100Config::AbstractParamDescriptionConstPtr(new argos3d_p100Config::ParamDescription<double>("Amplitude_Threshold", "double", 0, "What should be the amplitude filter threshold. Image pixels with lesser aplitude values will be filtered out. Amplitude Filter Status should be true to use this filter.", "", &argos3d_p100Config::Amplitude_Threshold)));
//#line 233 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 233 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(argos3d_p100Config::AbstractGroupDescriptionConstPtr(new argos3d_p100Config::GroupDescription<argos3d_p100Config::DEFAULT, argos3d_p100Config>(Default)));
//#line 390 "/opt/ros/hydro/share/dynamic_reconfigure/templates/ConfigType.h.template"
    
      for (std::vector<argos3d_p100Config::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<argos3d_p100Config::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<argos3d_p100Config::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    argos3d_p100Config __max__;
    argos3d_p100Config __min__;
    argos3d_p100Config __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const argos3d_p100ConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static argos3d_p100ConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &argos3d_p100Config::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const argos3d_p100Config &argos3d_p100Config::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const argos3d_p100Config &argos3d_p100Config::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const argos3d_p100Config &argos3d_p100Config::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<argos3d_p100Config::AbstractParamDescriptionConstPtr> &argos3d_p100Config::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<argos3d_p100Config::AbstractGroupDescriptionConstPtr> &argos3d_p100Config::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const argos3d_p100ConfigStatics *argos3d_p100Config::__get_statics__()
  {
    const static argos3d_p100ConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = argos3d_p100ConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __ARGOS3D_P100RECONFIGURATOR_H__
