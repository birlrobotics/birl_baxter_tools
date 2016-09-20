#include "copm_display.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "copm_visual.h"

namespace copm_plugin
{
	CopmDisplay::CopmDisplay()
	{
		color_property_ = new rviz::ColorProperty("Color", QColor(255, 0, 0),
												  "Color to draw the Center of Pressure/Mass marker",
												  this, SLOT(updateColorAndAlpha()));

		alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque",
												  this, SLOT(updateColorAndAlpha()));

		scale_property_ = new rviz::FloatProperty("Scale", 0.01, "How big the marker should be drawn",
												  this, SLOT(updateScale()));

		history_length_property_ = new rviz::IntProperty("Number of Markers", 1,
														 "How many previous markers to keep",
														 this, SLOT(updateHistoryLength()));

		
	}

	// instantiate all of the class stuff here
	void CopmDisplay::onInitialize()
	{
		MFDClass::onInitialize();
		updateHistoryLength();

	}

	CopmDisplay::~CopmDisplay()
	{}

	// clear any visuals here
	void CopmDisplay::reset()
	{
		MFDClass::reset();
		markers_.clear();
	}

	void CopmDisplay::updateColorAndAlpha()
	{
		float alpha = alpha_property_->getFloat();
		Ogre::ColourValue color = color_property_->getOgreColor();

		for(int i = 0; i < (int)markers_.size(); i++)
		{
			markers_[i]->setColor(color.r, color.g, color.b, alpha);
		}
	}

	void CopmDisplay::updateScale()
	{
		float scale = scale_property_->getFloat();

		for(int i = 0; i < (int)markers_.size(); i++)
		{
			markers_[i]->setScale(scale);
		}
	}

	void CopmDisplay::updateHistoryLength()
	{
		markers_.rset_capacity(history_length_property_->getInt());
	}

	void CopmDisplay::processMessage(const geometry_msgs::Vector3StampedConstPtr& msg)
	{
		Ogre::Quaternion orientation;
		Ogre::Vector3 position;
		if(!context_->getFrameManager()->getTransform(msg->header.frame_id,
													  msg->header.stamp,
													  position, orientation))
		{
			ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
					  msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
			return;
		}

		position.x = msg->vector.x;
		position.y = msg->vector.y;
		position.z = msg->vector.z;

		boost::shared_ptr<CopmVisual> marker;
		if(markers_.full())
		{
			marker = markers_.front();
		}
		else
		{
			marker.reset(new CopmVisual(context_->getSceneManager(), scene_node_));
		}

		marker->setMessage(msg);
		marker->setFramePosition(position);
		marker->setFrameOrientation(orientation);

		float alpha = alpha_property_->getFloat();
		Ogre::ColourValue color = color_property_->getOgreColor();
		marker->setColor(color.r, color.g, color.b, alpha);

		marker->setScale(scale_property_->getFloat());

		markers_.push_back(marker);
	}
}

// Tell pluginlib about this class
// needs to be done in the global scope, outside of any namespaces
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(copm_plugin::CopmDisplay, rviz::Display)
