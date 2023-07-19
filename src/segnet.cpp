/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

// imports
#include "std_msgs/Int8.h"
#include <stdio.h>
#include "ros/ros.h"

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-inference/segNet.h>

#include <unordered_map>
#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <mavros_msgs/Altitude.h>

#include <math.h>
/*
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

*/

// globals
sensor_msgs::Image classFrame;
mavros_msgs::Altitude droneAltitude;
segNet *net = NULL;

segNet::FilterMode overlay_filter = segNet::FILTER_LINEAR;
segNet::FilterMode mask_filter = segNet::FILTER_LINEAR;

imageConverter *input_cvt = NULL;
imageConverter *overlay_cvt = NULL;
imageConverter *mask_color_cvt = NULL;
imageConverter *mask_class_cvt = NULL;

Publisher<sensor_msgs::Image> overlay_pub = NULL;
Publisher<sensor_msgs::Image> mask_color_pub = NULL;
Publisher<sensor_msgs::Image> mask_class_pub = NULL;
Publisher<std_msgs::Int8> safe_to_land_pub = NULL;

Publisher<vision_msgs::VisionInfo> info_pub = NULL;
vision_msgs::VisionInfo info_msg;

// triggered when a new subscriber connected
void info_callback()
{
	ROS_INFO("new subscriber connected to vision_info topic, sending VisionInfo msg");
	info_pub->publish(info_msg);
}

// publish overlay image
bool publish_overlay(uint32_t width, uint32_t height)
{
	// assure correct image size
	if (!overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat))
		return false;

	// generate the overlay
	if (!net->Overlay(overlay_cvt->ImageGPU(), width, height, overlay_filter))
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if (!overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat))
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message
	overlay_pub->publish(msg);
}

// publish color mask
bool publish_mask_color(uint32_t width, uint32_t height)
{
	// assure correct image size
	if (!mask_color_cvt->Resize(width, height, imageConverter::ROSOutputFormat))
		return false;

	// generate the overlay
	if (!net->Mask(mask_color_cvt->ImageGPU(), width, height, mask_filter))
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if (!mask_color_cvt->Convert(msg, imageConverter::ROSOutputFormat))
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message
	mask_color_pub->publish(msg);
}

// publish class mask
bool publish_mask_class(uint32_t width, uint32_t height)
{
	// assure correct image size
	if (!mask_class_cvt->Resize(width, height, IMAGE_GRAY8))
		return false;

	// generate the overlay
	if (!net->Mask((uint8_t *)mask_class_cvt->ImageGPU(), width, height))
		return false;
	
	// populate the message
	sensor_msgs::Image msg;

	//I think this turns to a ros image
	if (!mask_class_cvt->Convert(msg, IMAGE_GRAY8))
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message
	classFrame = msg;
	mask_class_pub->publish(msg);
}


//move into another file
// might break up into multiple methods that are called later
void publish_is_safe_to_land()
{

	//cv::Mat cv_image(cv::Size(1024, 512), IMAGE_GRAY8, imgCUDA);
	// converting to open cv
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(classFrame); //, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	std_msgs::Int8 msg;
	msg.data = 1;
	_Float32 D = 1.8; //should be 1.8 // diameter of drone in meters
	cv::Mat matrix = cv_ptr->image;
	_Float32 theta = 69 * (M_PI / 180); // radians horizantalFOV
	_Float32 phi = 42 * (M_PI / 180);	// degrees verticalFOV
	int Rx = matrix.cols;
	int Ry = matrix.rows;
	_Float32 alt = droneAltitude.local; // local altitude of drone in meters

	// might need to do some casting above

	_Float32 Px = (2 * alt * tan(theta)) / (Rx); //passes sanity check
	_Float32 Py = (2 * alt * tan(phi)) / (Ry);

	int Lx = ceil((D / 2) * (1 / Px)); // bounds in x
	int Ly = ceil((D / 2) * (1 / Py)); // bounds in y

	//printf("Lx is %d\n", Lx);
	//printf("Ly is %d\n", Ly);
	// getting row col data
	uint8_t *pixelPtr = (uint8_t *)matrix.data;
	int cn = matrix.channels();                            

	int typeOfObj;

	int centerX = matrix.cols / 2;
	int centerY = matrix.rows / 2;
	

	for (int i = (centerX - Lx); i < (centerX + Lx); i++)
	{
		for (int j = (centerY - Ly); j < (centerY + Ly); j++)
		{
			typeOfObj = pixelPtr[i * matrix.cols * cn + j * cn];
			if (typeOfObj == net->FindClassID("road"))
			{
				msg.data = 0;
				break;
			}
		}
		if (msg.data == 0)
		{
			break;
		}
	}

	safe_to_land_pub->publish(msg);
}

// input image subscriber callback
void img_callback(const sensor_msgs::ImageConstPtr input)
{
	//ROS_INFO("inside image callback\n");
	// convert the image to reside on GPU
	if (!input_cvt || !input_cvt->Convert(input))
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;
	}

	// process the segmentation network
	//converts to CUDA
	if (!net->Process(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight()))
	{
		ROS_ERROR("failed to process segmentation on %ux%u image", input->width, input->height);
		return;
	}

	// color overlay
	if (ROS_NUM_SUBSCRIBERS(overlay_pub) > 0)
		publish_overlay(input->width, input->height);

	// color mask
	if (ROS_NUM_SUBSCRIBERS(mask_color_pub) > 0)
		publish_mask_color(input->width, input->height);


	//publish_mask_class(net->GetGridWidth(), net->GetGridHeight()); //mask is different resolution from orignial image
	//this will fuck preformance probably
	publish_mask_class(input->width, input->height); //mask is different resolution from orignial image
	publish_is_safe_to_land();

}

void alt_callback(const mavros_msgs::Altitude altitude_msg)
{
	droneAltitude = altitude_msg;

	//printf("The altitude inside the alt callback is %f", altitude_msg.local)
	//ROS_INFO("inside alt callback");
}

// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("segnet");

	/*
	 * declare parameters
	 */
	std::string model_name = "fcn-resnet18-cityscapes-1024x512"; // fcn-resnet18-cityscapes-2048x1024";
	std::string model_path;
	std::string prototxt_path;
	std::string class_labels_path;
	std::string class_colors_path;

	std::string input_blob = SEGNET_DEFAULT_INPUT;
	std::string output_blob = SEGNET_DEFAULT_OUTPUT;

	std::string mask_filter_str = "linear";
	std::string overlay_filter_str = "linear";

	float overlay_alpha = 180.0f;

	ROS_DECLARE_PARAMETER("model_name", model_name);
	ROS_DECLARE_PARAMETER("model_path", model_path);
	ROS_DECLARE_PARAMETER("prototxt_path", prototxt_path);
	ROS_DECLARE_PARAMETER("class_labels_path", class_labels_path);
	ROS_DECLARE_PARAMETER("class_colors_path", class_colors_path);
	ROS_DECLARE_PARAMETER("input_blob", input_blob);
	ROS_DECLARE_PARAMETER("output_blob", output_blob);
	ROS_DECLARE_PARAMETER("mask_filter", mask_filter_str);
	ROS_DECLARE_PARAMETER("overlay_filter", overlay_filter_str);
	ROS_DECLARE_PARAMETER("overlay_alpha", overlay_alpha);

	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("model_name", model_name);
	ROS_GET_PARAMETER("model_path", model_path);
	ROS_GET_PARAMETER("prototxt_path", prototxt_path);
	ROS_GET_PARAMETER("class_labels_path", class_labels_path);
	ROS_GET_PARAMETER("class_colors_path", class_colors_path);
	ROS_GET_PARAMETER("input_blob", input_blob);
	ROS_GET_PARAMETER("output_blob", output_blob);
	ROS_GET_PARAMETER("mask_filter", mask_filter_str);
	ROS_GET_PARAMETER("overlay_filter", overlay_filter_str);
	ROS_GET_PARAMETER("overlay_alpha", overlay_alpha);

	overlay_filter = segNet::FilterModeFromStr(overlay_filter_str.c_str(), segNet::FILTER_LINEAR);
	mask_filter = segNet::FilterModeFromStr(mask_filter_str.c_str(), segNet::FILTER_LINEAR);

	/*
	 * load segmentation network
	 */
	if (model_path.size() > 0)
	{
		// create network using custom model paths
		net = segNet::Create(prototxt_path.c_str(), model_path.c_str(),
							 class_labels_path.c_str(), class_colors_path.c_str(),
							 input_blob.c_str(), output_blob.c_str());
	}
	else
	{
		// create network using the built-in model
		net = segNet::Create(model_name.c_str());
	}

	if (!net)
	{
		ROS_ERROR("failed to load segNet model");
		return 0;
	}

	// set alpha blending value for classes that don't explicitly already have an alpha
	net->SetOverlayAlpha(overlay_alpha);


	//setting class color
	net->SetClassColor( (uint32_t )  net->FindClassID("road"), 255.0, 255.0, 0.0, 0.0 ); //yellow
	net->SetClassColor( (uint32_t )  net->FindClassID("car"), 255.0 , 0.0, 0.0, 0.0 ); //red
	net->SetClassColor ( (uint32_t ) net->FindClassID("terrain"), 0.0, 255.0, 0.0, 0.0); //green

	/*
	 * create the class labels parameter vector
	 */
	std::hash<std::string> model_hasher; // hash the model path to avoid collisions on the param server
	std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());
	const size_t model_hash = model_hasher(model_hash_str);

	ROS_INFO("model hash => %zu", model_hash);
	ROS_INFO("hash string => %s", model_hash_str.c_str());

	// obtain the list of class descriptions
	std::vector<std::string> class_descriptions;
	const uint32_t num_classes = net->GetNumClasses();

	for (uint32_t n = 0; n < num_classes; n++)
	{
		const char *label = net->GetClassDesc(n);

		if (label != NULL)
			class_descriptions.push_back(label);
	}

	// create the key on the param server
	std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

	ROS_DECLARE_PARAMETER(class_key, class_descriptions);
	ROS_SET_PARAMETER(class_key, class_descriptions);

	// populate the vision info msg
	std::string node_namespace = ROS_GET_NAMESPACE();
	ROS_INFO("node namespace => %s", node_namespace.c_str());

	info_msg.database_location = node_namespace + std::string("/") + class_key;
	info_msg.database_version = 0;
	info_msg.method = net->GetModelPath();

	ROS_INFO("class labels => %s", info_msg.database_location.c_str());

	/*
	 * create image converters
	 */
	input_cvt = new imageConverter();
	overlay_cvt = new imageConverter();
	mask_color_cvt = new imageConverter();
	mask_class_cvt = new imageConverter();

	if (!input_cvt || !overlay_cvt || !mask_color_cvt || !mask_class_cvt)
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}


	/*
	 * subscribe to image topic
	 */

	// auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);
	auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "/camera/color/image_raw", 5, img_callback);

	// Subing to altitude
	auto alt_sub = ROS_CREATE_SUBSCRIBER(mavros_msgs::Altitude, "/mavros/altitude", 1, alt_callback);


	/*
	 * advertise publisher topics
	 */

	ROS_CREATE_PUBLISHER(std_msgs::Int8, "is_safe_to_land", 2, safe_to_land_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "color_mask", 2, mask_color_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "class_mask", 2, mask_class_pub); // is 8 bit single channel image with each pixel as class ID

	ROS_CREATE_PUBLISHER_STATUS(vision_msgs::VisionInfo, "vision_info", 1, info_callback, info_pub);


	/*
	 * wait for messages
	 */
	ROS_INFO("safeland_cv node initialized, waiting for messages");
	ROS_SPIN();

	/*
	 * free resources
	 */
	delete net;
	delete input_cvt;
	delete overlay_cvt;
	delete mask_color_cvt;
	delete mask_class_cvt;
	// delete classFrame;
	// delete droneAltitude;

	return 0;
}
