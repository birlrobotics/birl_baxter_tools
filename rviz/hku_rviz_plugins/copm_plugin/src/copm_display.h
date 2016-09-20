#ifndef COPM_PLUGIN_COPM_DISPLAY_H_
#define COPM_PLUGIN_COPM_DISPLAY_H_

#include <rviz/message_filter_display.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <boost/circular_buffer.hpp>

// forward declarations
namespace Ogre {
	class SceneNode;
}

namespace rviz {
	class ColorProperty;
	class FloatProperty;
	class IntProperty;
}

namespace copm_plugin {

	class CopmVisual;

	class CopmDisplay : public rviz::MessageFilterDisplay<geometry_msgs::Vector3Stamped>
	{
	Q_OBJECT

	public:
		CopmDisplay();
		virtual ~CopmDisplay();

	protected:
		virtual void onInitialize();

		virtual void reset();

	private Q_SLOTS:
		void updateColorAndAlpha();

		void updateScale();
		
		void updateHistoryLength();

	private:
		void processMessage(const geometry_msgs::Vector3StampedConstPtr& msg);

		rviz::ColorProperty* color_property_;
		rviz::FloatProperty* alpha_property_;
		rviz::FloatProperty* scale_property_;
		rviz::IntProperty* history_length_property_;

		boost::circular_buffer<boost::shared_ptr<CopmVisual> > markers_;
	};
	
}

#endif
