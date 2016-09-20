#ifndef COPM_PLUGIN_COPM_VISUAL_H_
#define COPM_PLUGIN_COPM_VISUAL_H_

#include <geometry_msgs/Vector3Stamped.h>

namespace Ogre
{
	class Vector3;
	class Quaternion;
	class SceneManager;
	class SceneNode;
}

namespace rviz
{
	class Shape;
}

namespace copm_plugin
{
	class CopmVisual
	{
	public:
		CopmVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

		virtual ~CopmVisual();

		void setMessage(const geometry_msgs::Vector3StampedConstPtr& msg);

		void setFramePosition(const Ogre::Vector3& position);
		void setFrameOrientation(const Ogre::Quaternion& orientation);

		void setScale(float scale);
		void setColor(float r, float g, float b, float a);

	private:
		

		// object implementing the sphere shape
		boost::shared_ptr<rviz::Shape> copm_marker_;

		// A scenenode whose pose is set to match the coordinate frame of the
		// Copm message header
		Ogre::SceneNode* frame_node_;

		Ogre::SceneManager* scene_manager_;
	};
}

#endif
