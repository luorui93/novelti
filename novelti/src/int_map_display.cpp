/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/custom_parameter_indices.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "novelti/map_display_base.h"
#include "novelti/IntMap.h"

//using namespace rviz;

namespace rviz
{
class IntMapDisplay : public MapDisplayBase<novelti::IntMap>

IntMapDisplay::IntMapDisplay()
  : Display()
  , manual_object_( NULL )
  , loaded_( false )
  , resolution_( 0.0f )
  , width_( 0 )
  , height_( 0 )
{
  connect(this, SIGNAL( mapUpdated() ), this, SLOT( showMap() ));
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<novelti::IntMap>() ),
                                          "novelti::IntMap topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  alpha_property_ = new FloatProperty( "Alpha", 0.7,
                                       "Amount of transparency to apply to the map.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  color_scheme_property_ = new EnumProperty( "Color Scheme", "map", "How to color the occupancy values.",
                                             this, SLOT( updatePalette() ));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize() below.
  color_scheme_property_->addOption( "map", 0 );
  color_scheme_property_->addOption( "costmap", 1 );
  color_scheme_property_->addOption( "regions", 2 );
  color_scheme_property_->addOption( "distance", 3 );
  ///color_scheme_property_->addOption( "intensity", 3 );

  draw_under_property_ = new Property( "Draw Behind", false,
                                       "Rendering option, controls whether or not the map is always"
                                       " drawn behind everything else.",
                                       this, SLOT( updateDrawUnder() ));

  resolution_property_ = new FloatProperty( "Resolution", 0,
                                            "Resolution of the map. (not editable)", this );
  resolution_property_->setReadOnly( true );

  width_property_ = new IntProperty( "Width", 0,
                                     "Width of the map, in meters. (not editable)", this );
  width_property_->setReadOnly( true );
  
  height_property_ = new IntProperty( "Height", 0,
                                      "Height of the map, in meters. (not editable)", this );
  height_property_->setReadOnly( true );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                           "Position of the bottom left corner of the map, in meters. (not editable)",
                                           this );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                  "Orientation of the map. (not editable)",
                                                  this );
  orientation_property_->setReadOnly( true );
}

unsigned char* makeIntMapPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;
  // Standard gray map palette values
  for( int i = 0; i <= 100; i++ )
  {
    unsigned char v = 255 - (255 * i) / 100;
    *palette_ptr++ = v; // red
    *palette_ptr++ = v; // green
    *palette_ptr++ = v; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal positive values in green
  for( int i = 101; i <= 127; i++ )
  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for( int i = 128; i <= 254; i++ )
  {
    *palette_ptr++ = 255; // red
    *palette_ptr++ = (255*(i-128))/(254-128); // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70; // red
  *palette_ptr++ = 0x89; // green
  *palette_ptr++ = 0x86; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}

unsigned char* makeCostmapPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;

  // zero values have alpha=0
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 0; // alpha

  // Blue to red spectrum for most normal cost values
  for( int i = 1; i <= 98; i++ )
  {
    unsigned char v = (255 * i) / 100;
    *palette_ptr++ = v; // red
    *palette_ptr++ = 0; // green
    *palette_ptr++ = 255 - v; // blue
    *palette_ptr++ = 255; // alpha
  }
  // inscribed obstacle values (99) in cyan
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 255; // green
  *palette_ptr++ = 255; // blue
  *palette_ptr++ = 255; // alpha
  // lethal obstacle values (100) in purple
  *palette_ptr++ = 255; // red
  *palette_ptr++ = 255; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 255; // alpha
  // illegal positive values in green
  for( int i = 101; i <= 127; i++ )
  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for( int i = 128; i <= 254; i++ )
  {
    *palette_ptr++ = 255; // red
    *palette_ptr++ = (255*(i-128))/(254-128); // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70; // red
  *palette_ptr++ = 0x89; // green
  *palette_ptr++ = 0x86; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}

unsigned char* makeRegionPalette() {
    
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;
  //region colors
  for( unsigned char i = 0; i < 20; i++ )  {
        //yellow FFF506
        *palette_ptr++ = 255; // red
        *palette_ptr++ = 245; // green
        *palette_ptr++ = 6; // blue
        *palette_ptr++ = 255; // alpha
        
        //dark blue 7506eb
        *palette_ptr++ = 117; // red
        *palette_ptr++ = 6; // green
        *palette_ptr++ = 235; // blue
        *palette_ptr++ = 255; // alpha
        
        //orange e8880c
        *palette_ptr++ = 232; // red
        *palette_ptr++ = 136; // green
        *palette_ptr++ = 12; // blue
        *palette_ptr++ = 255; // alpha
        
        //light blue
        *palette_ptr++ = 82; // red
        *palette_ptr++ = 176; // green
        *palette_ptr++ = 255; // blue
        *palette_ptr++ = 255; // alpha
        
        //green
        *palette_ptr++ = 0; // red
        *palette_ptr++ = 255; // green
        *palette_ptr++ = 65; // blue
        *palette_ptr++ = 255; // alpha
        
        //red
        *palette_ptr++ = 255; // red
        *palette_ptr++ = 0; // green
        *palette_ptr++ = 0; // blue
        *palette_ptr++ = 255; // alpha
        
        
        
        //yellow FFF506
        *palette_ptr++ = 115; // red
        *palette_ptr++ = 125; // green
        *palette_ptr++ = 6; // blue
        *palette_ptr++ = 255; // alpha
        
        //dark blue 7506eb
        *palette_ptr++ = 117; // red
        *palette_ptr++ = 6; // green
        *palette_ptr++ = 115; // blue
        *palette_ptr++ = 255; // alpha
        
        //orange e8880c
        *palette_ptr++ = 122; // red
        *palette_ptr++ = 136; // green
        *palette_ptr++ = 12; // blue
        *palette_ptr++ = 255; // alpha
        
        //light blue
        *palette_ptr++ = 82; // red
        *palette_ptr++ = 176; // green
        *palette_ptr++ = 115; // blue
        *palette_ptr++ = 255; // alpha
        
        //green
        *palette_ptr++ = 0; // red
        *palette_ptr++ = 115; // green
        *palette_ptr++ = 65; // blue
        *palette_ptr++ = 255; // alpha
        
        //red
        *palette_ptr++ = 115; // red
        *palette_ptr++ = 0; // green
        *palette_ptr++ = 0; // blue
        *palette_ptr++ = 255; // alpha
  }              
  // illegal negative (char) values in shades of red/yellow
  for( int i = 240; i <= 254; i++ )  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 0; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal value 255 transparent
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 0; // alpha

  return palette;
}

unsigned char* makeDistancePalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;
  
  *palette_ptr++ = 100; // red
  *palette_ptr++ = 100; // green
  *palette_ptr++ = 255; // blue
  *palette_ptr++ = 255; // alpha
  *palette_ptr++ = 100; // red
  *palette_ptr++ = 100; // green
  *palette_ptr++ = 255; // blue
  *palette_ptr++ = 255; // alpha   
  
  //light grey
  *palette_ptr++ = 240; // red
  *palette_ptr++ = 240; // green
  *palette_ptr++ = 240; // blue
  *palette_ptr++ = 255; // alpha
  *palette_ptr++ = 240; // red
  *palette_ptr++ = 240; // green
  *palette_ptr++ = 240; // blue
  *palette_ptr++ = 255; // alpha
  for( int i = 0; i < 4; i++ ) {
    //darker grey
    *palette_ptr++ = 225; // red
    *palette_ptr++ = 225; // green
    *palette_ptr++ = 225; // blue
    *palette_ptr++ = 255; // alpha
    *palette_ptr++ = 225; // red
    *palette_ptr++ = 225; // green
    *palette_ptr++ = 225; // blue
    *palette_ptr++ = 255; // alpha
    //light grey
    *palette_ptr++ = 240; // red
    *palette_ptr++ = 240; // green
    *palette_ptr++ = 240; // blue
    *palette_ptr++ = 255; // alpha
    *palette_ptr++ = 240; // red
    *palette_ptr++ = 240; // green
    *palette_ptr++ = 240; // blue
    *palette_ptr++ = 255; // alpha
  }
 
  for( int i = 20; i < 254; i++ ) {
    //light grey
    *palette_ptr++ = 255; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  
  // 254 valid obstacles
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 255; // alpha
  
  // 255 invalid unvisited cell
  *palette_ptr++ = 255; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}


Ogre::TexturePtr makeIntMapPaletteTexture( unsigned char *palette_bytes )
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.bind( new Ogre::MemoryDataStream( palette_bytes, 256*4 ));

  static int palette_tex_count = 0;
  std::stringstream ss;
  ss << "IntMapPaletteTexture" << palette_tex_count++;
  return Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                           palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0 );
}

void IntMapDisplay::onInitialize()
{
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back( makeIntMapPaletteTexture( makeIntMapPalette() ));
  color_scheme_transparency_.push_back( false );
  palette_textures_.push_back( makeIntMapPaletteTexture( makeCostmapPalette() ));
  color_scheme_transparency_.push_back( true );
  palette_textures_.push_back( makeIntMapPaletteTexture( makeRegionPalette() ));  
  color_scheme_transparency_.push_back( true );
  palette_textures_.push_back( makeIntMapPaletteTexture( makeDistancePalette() ));  
  color_scheme_transparency_.push_back( true );

  // Set up map material
  static int material_count = 0;
  std::stringstream ss;
  ss << "IntMapMaterial" << material_count++;
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone( ss.str() );

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias( -16.0f, 0.0f );
  material_->setCullingMode( Ogre::CULL_NONE );
  material_->setDepthWriteEnabled(false);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "IntMapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject( ss2.str() );
  scene_node_->attachObject( manual_object_ );

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( 1.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top left
      manual_object_->position( 0.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      manual_object_->position( 1.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( 1.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();

  if( draw_under_property_->getValue().toBool() )
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible( false );

  updateAlpha();
}

void IntMapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if( !topic_property_->getTopic().isEmpty() )
  {
    try
    {
      map_sub_ = update_nh_.subscribe( topic_property_->getTopicStd(), 1, &IntMapDisplay::incomingMap, this );
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
    }

  }
}

// helper class to set alpha parameter on all renderables.
class AlphaSetter: public Ogre::Renderable::Visitor
{
public:
  AlphaSetter( float alpha )
  : alpha_vec_( alpha, alpha, alpha, alpha )
  {}

  void visit( Ogre::Renderable *rend, ushort lodIndex, bool isDebug, Ogre::Any *pAny=0)
  {
    rend->setCustomParameter( ALPHA_PARAMETER, alpha_vec_ );
  }
private:
  Ogre::Vector4 alpha_vec_;
};

bool validateFloats(const novelti::IntMap& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.info.resolution );
  valid = valid && validateFloats( msg.info.origin );
  return valid;
}

void IntMapDisplay::showMap()
{
  if (current_map_.data.empty())
  {
    return;
  }

  if( !validateFloats( current_map_ ))
  {
    setStatus( StatusProperty::Error, "Map", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  if( current_map_.info.width * current_map_.info.height == 0 )
  {
    std::stringstream ss;
    ss << "Map is zero-sized (" << current_map_.info.width << "x" << current_map_.info.height << ")";
    setStatus( StatusProperty::Error, "Map", QString::fromStdString( ss.str() ));
    return;
  }

  setStatus( StatusProperty::Ok, "Message", "Map received" );

  ROS_DEBUG( "Received a %d X %d map @ %.3f m/pix\n",
             current_map_.info.width,
             current_map_.info.height,
             current_map_.info.resolution );

  float resolution = current_map_.info.resolution;

  int width = current_map_.info.width;
  int height = current_map_.info.height;


  Ogre::Vector3 position( current_map_.info.origin.position.x,
                          current_map_.info.origin.position.y,
                          current_map_.info.origin.position.z );
  Ogre::Quaternion orientation( current_map_.info.origin.orientation.w,
                                current_map_.info.origin.orientation.x,
                                current_map_.info.origin.orientation.y,
                                current_map_.info.origin.orientation.z );
  frame_ = current_map_.header.frame_id;
  if (frame_.empty())
  {
    frame_ = "/map";
  }

  unsigned int pixels_size = width * height;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if( pixels_size != current_map_.data.size() )
  {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width
       << ", height = " << height << ", data size = " << current_map_.data.size();
    setStatus( StatusProperty::Error, "Map", QString::fromStdString( ss.str() ));
    map_status_set = true;

    // Keep going, but don't read past the end of the data.
    if( current_map_.data.size() < pixels_size )
    {
      num_pixels_to_copy = current_map_.data.size();
    }
  }
  
  if (color_scheme_property_->getOptionInt()==3) { //distance map
    for (unsigned int k=0;k<pixels_size;k++) {
      pixels[k] = 
        current_map_.data[k]>=0   ? static_cast<unsigned char>(current_map_.data[k]%20): //valid distnace
        current_map_.data[k]==-10 ? 254 : //valid obstacles
                                    255 ;  //invalid unvisited cell
    }
  } else {
    for (unsigned int k=0;k<pixels_size;k++) {
      pixels[k] = static_cast<unsigned char>(current_map_.data[k]);      
    } 
  }
  //memcpy( pixels, &current_map_.data[0], num_pixels_to_copy );

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind( new Ogre::MemoryDataStream( pixels, pixels_size ));

  if( !texture_.isNull() )
  {
    Ogre::TextureManager::getSingleton().remove( texture_->getName() );
    texture_.setNull();
  }

  static int tex_count = 0;
  std::stringstream ss;
  ss << "IntMapTexture" << tex_count++;
  try
  {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                 pixel_stream, width, height, Ogre::PF_L8, Ogre::TEX_TYPE_2D,
                                                                 0);

    if( !map_status_set )
    {
      setStatus( StatusProperty::Ok, "Map", "Map OK" );
    }
  }
  catch(Ogre::RenderingAPIException&)
  {
    Ogre::Image image;
    pixel_stream->seek(0);
    float fwidth = width;
    float fheight = height;
    if( width > height )
    {
      float aspect = fheight / fwidth;
      fwidth = 2048;
      fheight = fwidth * aspect;
    }
    else
    {
      float aspect = fwidth / fheight;
      fheight = 2048;
      fwidth = fheight * aspect;
    }

    {
      std::stringstream ss;
      ss << "Map is larger than your graphics card supports.  Downsampled from [" << width << "x" << height << "] to [" << fwidth << "x" << fheight << "]";
      setStatus(StatusProperty::Ok, "Map", QString::fromStdString( ss.str() ));
    }

    ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures of size > 2048.  Downsampling to [%d x %d]...", (int)fwidth, (int)fheight);
    //ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]", pixel_stream->size(), width, height, width * height);
    image.loadRawData(pixel_stream, width, height, Ogre::PF_L8);
    image.resize(fwidth, fheight, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }

  delete [] pixels;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  updatePalette();

  resolution_property_->setValue( resolution );
  width_property_->setValue( width );
  height_property_->setValue( height );
  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );

  transformMap();
  manual_object_->setVisible( true );
  scene_node_->setScale( resolution * width, resolution * height, 1.0 );

  context_->queueRender();
}

void IntMapDisplay::transformMap()
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(frame_, ros::Time(), current_map_.info.origin, position, orientation))
  {
    ROS_DEBUG( "Error transforming map '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), frame_.c_str(), qPrintable( fixed_frame_ ));

    setStatus( StatusProperty::Error, "Transform",
               "No transform from [" + QString::fromStdString( frame_ ) + "] to [" + fixed_frame_ + "]" );
  }
  else
  {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::IntMapDisplay, rviz::Display )
