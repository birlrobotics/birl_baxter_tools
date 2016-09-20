#include "copm_visual.h"
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

namespace copm_plugin
{
	CopmVisual::CopmVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
	{
		scene_manager_ = scene_manager;

		frame_node_ = parent_node->createChildSceneNode();

		copm_marker_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
	}

	CopmVisual::~CopmVisual()
	{
		// Destroy the frame since it is no longer needed
		scene_manager_->destroySceneNode(frame_node_);
	}

	void CopmVisual::setMessage(const geometry_msgs::Vector3StampedConstPtr& msg)
	{
		Ogre::Vector3 scale(.01, .01, .01);

		copm_marker_->setScale(scale);
	}

	void CopmVisual::setFramePosition(const Ogre::Vector3& position)
	{
		frame_node_->setPosition(position);
	}

	void CopmVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
	{
		frame_node_->setOrientation(orientation);
	}

	void CopmVisual::setScale(float scale)
	{
		Ogre::Vector3 scale_vec(scale, scale, scale);
		copm_marker_->setScale(scale_vec);
	}

	void CopmVisual::setColor(float r, float g, float b, float a)
	{
		copm_marker_->setColor(r,g,b,a);
	}

}
