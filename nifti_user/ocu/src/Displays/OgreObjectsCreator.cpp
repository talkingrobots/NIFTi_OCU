// Benoit 2011-06-16

#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreNode.h>
#include <OGRE/OgreSceneManager.h>

#include <Displays/OgreObjectsCreator.h>

using namespace Ogre;

namespace eu
{
    namespace nifti
    {
        namespace ocu
        {
            namespace display
            {

                std::vector<Ogre::ColourValue> OgreObjectsCreator::colors = std::vector< Ogre::ColourValue >((size_t) 0); // size_t is due to a compiler problem with STL vector

                const Ogre::ColourValue& OgreObjectsCreator::getColor(u_int index)
                {
                    assert(index < 125);

                    if (colors.size() == 0)
                    {
                        initializeColors();
                    }

                    return colors.at(index);
                }

                void OgreObjectsCreator::initializeColors()
                {
                    // Creates a set of 125 distinct colors (make it better generated, to vary the colors more)
                    colors.reserve(128);
                    for (float r = 0.875; r > 0.250; r -= 0.125)
                    {
                        for (float g = 0.875; g > 0.250; g -= 0.125)
                        {
                            for (float b = 0.875; b > 0.250; b -= 0.125)
                            {
                                colors.push_back(Ogre::ColourValue(r, g, b, 1));
                            }
                        }
                    }
                }

                // Todo: throw the exception instead of catching it and let the caller deal with it
                void OgreObjectsCreator::createOgreGroundPolygon(const eu_nifti_env::Polygon& polygonROS, Ogre::ManualObject* polygonOgre, const Ogre::ColourValue& color, float defaultHeight, Ogre::Node* node)
                {
                    //std::cout << "IN void OgreObjectsCreator::createPolygon()" << std::endl;

                    try
                    {
                        polygonOgre->estimateVertexCount(polygonROS.points.size());
                        polygonOgre->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_FAN); // Draws filled triangles in order to fill the entire polygon

                        polygonOgre->colour(color);

                        std::vector<geometry_msgs::Point >::const_iterator it_begin = polygonROS.points.begin();
                        const std::vector<geometry_msgs::Point >::const_iterator it_end = polygonROS.points.end();

                        // Adds all vertices to the polygon object
                        for (; it_begin != it_end; it_begin++)
                        {
                            const geometry_msgs::Point& point = *(it_begin);
                            //std::cout << "BEFORE adding a vertex @ (" <<  point.x << ", " << point.y << ")" << std::endl;

                            // Ignores the z component
                            Ogre::Vector3 pos(point.x, point.y, defaultHeight);
                            
                            // If a node was provided, then convert the world coordinates (relative to the root scene node) to local ones (relative to the provided node)
                            if(node != NULL)
                            {
                                pos = node->convertWorldToLocalPosition(pos);
                            }

                            // Adds this vertex to the polygon
                            polygonOgre->position(pos);
                        }

                        polygonOgre->end();

                        //std::cout << "Bounding box radius : " <<  polygonOgre->getBoundingRadius() << std::endl;

                    } 
                    catch (Ogre::Exception& ex)
                    {
                        std::cerr << "An Ogre exception has occured while creating a polygon: " << ex.getFullDescription().c_str() << std::endl;
                    }

                    //std::cout << "OUT void OgreObjectsCreator::createPolygon()" << std::endl;
                }

                void OgreObjectsCreator::makeCircle(Ogre::ManualObject * circle, double radius, const Ogre::ColourValue& color)
                {
                    // From http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Circle3D

                    // accuracy is the count of points (and lines).
                    // Higher values make the circle smoother, but may slowdown the performance.
                    // The performance also is related to the count of circles.
                    const double accuracy = 35;

                    circle->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
                    circle->colour(color);

                    u_int pointIndex = 0;
                    for (double theta = 0; theta <= 2 * Ogre::Math::PI; theta += Ogre::Math::PI / accuracy)
                    {
                        circle->position(radius * cos(theta), radius * sin(theta), 0);
                        circle->index(pointIndex++);
                    }
                    circle->index(0); // Rejoins the last point to the first.

                    circle->end();
                }

                void OgreObjectsCreator::makeLine(Ogre::ManualObject *line, const Ogre::Vector3& point1, const Ogre::Vector3& point2, const Ogre::ColourValue& color)
                {
                    line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
                    line->colour(color);

                    line->position(point1);
                    line->position(point2);

                    line->end();
                }
                
                //From http://www.ogre3d.org/tikiwiki/ManualSphereMeshes&structure=Cookbook
                void OgreObjectsCreator::makeSphere(Ogre::ManualObject *sphere, const ColourValue& color, const float radius, const int nRings, const int nSegments)
                {
                    sphere->begin("BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST);
                    sphere->colour(color);

                    float fDeltaRingAngle = (Math::PI / nRings);
                    float fDeltaSegAngle = (2 * Math::PI / nSegments);
                    unsigned short wVerticeIndex = 0;

                    // Generate the group of rings for the sphere
                    for (int ring = 0; ring <= nRings; ring++)
                    {
                        float r0 = radius * sinf(ring * fDeltaRingAngle);
                        float y0 = radius * cosf(ring * fDeltaRingAngle);

                        // Generate the group of segments for the current ring
                        for (int seg = 0; seg <= nSegments; seg++)
                        {
                            float x0 = r0 * sinf(seg * fDeltaSegAngle);
                            float z0 = r0 * cosf(seg * fDeltaSegAngle);

                            // Add one vertex to the strip which makes up the sphere
                            sphere->position(x0, y0, z0);
                            sphere->normal(Vector3(x0, y0, z0).normalisedCopy());
                            sphere->textureCoord((float) seg / (float) nSegments, (float) ring / (float) nRings);

                            if (ring != nRings)
                            {
                                // each vertex (except the last) has six indicies pointing to it
                                sphere->index(wVerticeIndex + nSegments + 1);
                                sphere->index(wVerticeIndex);
                                sphere->index(wVerticeIndex + nSegments);
                                sphere->index(wVerticeIndex + nSegments + 1);
                                sphere->index(wVerticeIndex + 1);
                                sphere->index(wVerticeIndex);
                                wVerticeIndex++;
                            }
                        }; // end for seg
                    } // end for ring
                    sphere->end();
                    
//                    MeshPtr mesh = manual->convertToMesh(strName);
//                    mesh->_setBounds(AxisAlignedBox(Vector3(-radius, -radius, -radius), Vector3(radius, radius, radius)), false);
//
//                    mesh->_setBoundingSphereRadius(radius);
//                    unsigned short src, dest;
//                    if (!mesh->suggestTangentVectorBuildParams(VES_TANGENT, src, dest))
//                    {
//                        mesh->buildTangentVectors(VES_TANGENT, src, dest);
//                    }
                }
                
            }
        }
    }
}



